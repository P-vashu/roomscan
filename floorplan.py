#!/usr/bin/env python3
"""LiDAR tier, v0: fused point cloud -> rooms, walls, openings, rendered plan.

Usage:
    python floorplan.py data/raw/floor_only            # every 10th frame
    python floorplan.py data/raw/floor_only --step 5

Outputs in out/<scan>/: plan.png, plan.json, debug_walls.png, debug_free.png

Pipeline
  1. Fuse depth (scan_preview.fuse). World Y is gravity-up (ARKit).
  2. Manhattan alignment: rotate about Y to the angle that makes wall-band
     points pile up into the sharpest X/Z histograms.
  3. Wall map: 2 cm cells holding points in >= 3 of 4 height slices of a band
     above furniture height. Tall-and-thin = wall; beds and tables never reach it.
  4. Interior: floor-slab cells, closed and hole-filled together with walls.
  5. Rooms: distance transform of free space; cores wider than a doorway are
     seeds; seeds grow back over free space without crossing walls.
  6. Openings: where two rooms touch in free space, the contact length is the
     opening width.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage as ndi

from scan_preview import load_scan, pick_convention, fuse, floor_ceiling, plane_peak

PX = 0.02                 # grid cell size, metres
BAND = (1.3, 2.0)         # wall band, metres above floor (above most furniture)
FLOOR_TOL = 0.04          # floor slab half-thickness
SEED_HALF_WIDTH = 0.42    # free space must be wider than 2*this to seed a room
MIN_ROOM_M2 = 0.6
MIN_OPENING_M = 0.5


# ---------- alignment ----------
def rot_xz(pts, a):
    c, s = np.cos(a), np.sin(a)
    out = pts.copy()
    out[:, 0] = c * pts[:, 0] - s * pts[:, 2]
    out[:, 2] = s * pts[:, 0] + c * pts[:, 2]
    return out


def sharpness(xz, a):
    c, s = np.cos(a), np.sin(a)
    u = c * xz[:, 0] - s * xz[:, 1]
    v = s * xz[:, 0] + c * xz[:, 1]
    score = 0.0
    for w in (u, v):
        h, _ = np.histogram(w, bins=np.arange(w.min(), w.max() + PX, PX))
        score += float((h.astype(np.float64) ** 2).sum())
    return score


def manhattan_angle(band_pts):
    rng = np.random.default_rng(0)
    xz = band_pts[:, [0, 2]]
    if len(xz) > 300_000:
        xz = xz[rng.choice(len(xz), 300_000, replace=False)]
    coarse = np.deg2rad(np.arange(0, 90, 1.0))
    best = coarse[int(np.argmax([sharpness(xz, a) for a in coarse]))]
    fine = best + np.deg2rad(np.arange(-1, 1.01, 0.1))
    return float(fine[int(np.argmax([sharpness(xz, a) for a in fine]))])


# ---------- grids ----------
class Grid:
    def __init__(self, xz):
        self.origin = xz.min(0) - 0.3
        size = np.ceil((xz.max(0) + 0.3 - self.origin) / PX).astype(int) + 1
        self.W, self.H = int(size[0]), int(size[1])

    def cells(self, xz):
        ij = np.floor((xz - self.origin) / PX).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.W) & (ij[:, 1] >= 0) & (ij[:, 1] < self.H)
        return ij[ok]

    def occupancy(self, xz):
        img = np.zeros((self.H, self.W), np.int32)
        ij = self.cells(xz)
        np.add.at(img, (ij[:, 1], ij[:, 0]), 1)
        return img


def wall_map(pts, grid, floor_y, ceil_y):
    lo = floor_y + BAND[0]
    hi = floor_y + BAND[1]
    if ceil_y is not None:
        hi = min(hi, ceil_y - 0.15)
    edges = np.linspace(lo, hi, 5)
    hits = np.zeros((grid.H, grid.W), np.int32)
    for a, b in zip(edges[:-1], edges[1:]):
        sl = pts[(pts[:, 1] >= a) & (pts[:, 1] < b)]
        hits += (grid.occupancy(sl[:, [0, 2]]) > 0).astype(np.int32)
    walls = (hits >= 3).astype(np.uint8)
    walls = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    # drop specks
    n, lab, stats, _ = cv2.connectedComponentsWithStats(walls, connectivity=8)
    keep = np.zeros(n, bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] * PX * PX >= 0.01
    return keep[lab].astype(np.uint8)


def fill_holes(mask):
    return ndi.binary_fill_holes(mask > 0).astype(np.uint8)


def interior_map(pts, grid, floor_y, walls):
    slab = pts[np.abs(pts[:, 1] - floor_y) < FLOOR_TOL]
    floor = (grid.occupancy(slab[:, [0, 2]]) > 0).astype(np.uint8)
    floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    filled = fill_holes(floor | walls)
    free = filled & (1 - cv2.dilate(walls, np.ones((3, 3), np.uint8)))
    # keep free regions that actually contain observed floor
    n, lab = cv2.connectedComponents(free, connectivity=4)
    has_floor = np.bincount(lab[floor > 0], minlength=n) > 0
    has_floor[0] = False
    return has_floor[lab].astype(np.uint8), floor


# ---------- rooms ----------
def segment_rooms(free):
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * PX
    cores = (dist > SEED_HALF_WIDTH).astype(np.uint8)
    n, seeds = cv2.connectedComponents(cores, connectivity=4)
    sizes = np.bincount(seeds.ravel(), minlength=n) * PX * PX
    keep = np.zeros(n, bool)
    keep[1:] = sizes[1:] >= 0.3
    remap = np.zeros(n, np.int32)
    remap[keep] = np.arange(1, keep.sum() + 1)
    labels = remap[seeds]
    # geodesic growth inside free space (never crosses walls)
    fp = np.ones((3, 3), bool)
    for _ in range(2000):
        grown = ndi.grey_dilation(labels, footprint=fp)
        new = (labels == 0) & (free > 0) & (grown > 0)
        if not new.any():
            break
        labels[new] = grown[new]
    # drop tiny rooms
    areas = np.bincount(labels.ravel()) * PX * PX
    for r in range(1, len(areas)):
        if areas[r] < MIN_ROOM_M2:
            labels[labels == r] = 0
    ids = [r for r in np.unique(labels) if r > 0]
    remap = np.zeros(labels.max() + 1, np.int32)
    remap[ids] = np.arange(1, len(ids) + 1)
    return remap[labels]


def room_polygon(mask):
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)
    poly = cv2.approxPolyDP(c, 0.06 / PX, True)[:, 0, :].astype(float)
    return poly


def openings(labels):
    """Contact length between each pair of rooms, horizontal and vertical."""
    out = {}
    for a, b, axis in ((labels[:, :-1], labels[:, 1:], "v"), (labels[:-1, :], labels[1:, :], "h")):
        m = (a > 0) & (b > 0) & (a != b)
        for p, q in zip(a[m], b[m]):
            key = (int(min(p, q)), int(max(p, q)))
            out.setdefault(key, {"v": 0, "h": 0})[axis] += 1
    res = []
    for (p, q), c in out.items():
        width = max(c["v"], c["h"]) * PX
        if width >= MIN_OPENING_M:
            res.append({"rooms": [p, q], "width_m": round(width, 3)})
    return res


def opening_pixels(labels, p, q):
    a = labels
    m = np.zeros_like(a, bool)
    for dy, dx in ((0, 1), (1, 0)):
        x = a[: a.shape[0] - dy, : a.shape[1] - dx]
        y = a[dy:, dx:]
        hit = ((x == p) & (y == q)) | ((x == q) & (y == p))
        m[: a.shape[0] - dy, : a.shape[1] - dx] |= hit
    return m


# ---------- render ----------
def render(labels, walls, rooms, opens, path):
    rng = np.random.default_rng(3)
    pal = (rng.uniform(150, 235, (labels.max() + 1, 3))).astype(np.uint8)
    pal[0] = 255
    img = pal[labels]
    img[walls > 0] = (40, 40, 40)
    for o in opens:
        img[opening_pixels(labels, *o["rooms"])] = (0, 0, 220)
    for r in rooms:
        cx, cy = r["_centroid_px"]
        lines = [f"R{r['id']}", f"{r['floor_area_m2']:.1f} m2",
                 f"{r['bbox_m'][0]:.2f} x {r['bbox_m'][1]:.2f}"]
        if r["ceiling_height_m"] is not None:
            lines.append(f"h {r['ceiling_height_m']:.2f}")
        for k, t in enumerate(lines):
            cv2.putText(img, t, (int(cx) - 40, int(cy) - 20 + 18 * k),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    H = img.shape[0]
    cv2.line(img, (10, H - 10), (10 + int(1 / PX), H - 10), (0, 0, 255), 3)
    cv2.putText(img, "1 m", (10, H - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imwrite(str(path), img)


# ---------- main ----------
def plan_from_points(pts, cam_y, out_dir: Path):
    floor_y, ceil_y, _, _ = floor_ceiling(pts, cam_y)
    if floor_y is None:
        raise SystemExit("No floor found; cannot build a plan.")
    band = pts[(pts[:, 1] > floor_y + BAND[0]) & (pts[:, 1] < floor_y + BAND[1])]
    angle = manhattan_angle(band)
    pts = rot_xz(pts, angle)

    grid = Grid(pts[:, [0, 2]])
    walls = wall_map(pts, grid, floor_y, ceil_y)
    free, floor = interior_map(pts, grid, floor_y, walls)
    labels = segment_rooms(free)
    opens = openings(labels)

    # per-room ceiling heights (grid was built from all points, so all are in bounds)
    ceil_pts = pts[pts[:, 1] > cam_y.max() + 0.2]
    cij = np.floor((ceil_pts[:, [0, 2]] - grid.origin) / PX).astype(int)
    ceil_lab = labels[cij[:, 1], cij[:, 0]] if len(cij) else np.zeros(0, int)

    rooms = []
    for r in range(1, labels.max() + 1):
        m = labels == r
        ys, xs = np.nonzero(m)
        poly = room_polygon(m)
        edges = np.linalg.norm(np.roll(poly, -1, 0) - poly, axis=1) * PX
        ch = None
        if len(ceil_lab):
            yv = ceil_pts[ceil_lab == r, 1]
            if len(yv) > 2000:
                c, share = plane_peak(yv, yv.min() - 0.01, yv.max() + 0.01, len(yv))
                if c is not None and share > 0.02:
                    ch = round(c - floor_y, 3)
        rooms.append({
            "id": r,
            "floor_area_m2": round(float(m.sum() * PX * PX), 2),
            "bbox_m": [round(float((xs.max() - xs.min() + 1) * PX), 2),
                       round(float((ys.max() - ys.min() + 1) * PX), 2)],
            "polygon_m": (poly * PX + grid.origin).round(3).tolist(),
            "wall_lengths_m": [round(float(e), 2) for e in edges if e >= 0.3],
            "ceiling_height_m": ch,
            "_centroid_px": [float(xs.mean()), float(ys.mean())],
        })

    out_dir.mkdir(parents=True, exist_ok=True)
    render(labels, walls, rooms, opens, out_dir / "plan.png")
    cv2.imwrite(str(out_dir / "debug_walls.png"), 255 - walls * 255)
    dbg = np.full((grid.H, grid.W, 3), 255, np.uint8)
    dbg[floor > 0] = (200, 200, 200)
    dbg[free > 0] = (160, 220, 160)
    dbg[walls > 0] = (0, 0, 0)
    cv2.imwrite(str(out_dir / "debug_free.png"), dbg)

    result = {
        "tier": "lidar",
        "version": "v0",
        "floor_y_m": round(floor_y, 3),
        "global_ceiling_height_m": None if ceil_y is None else round(ceil_y - floor_y, 3),
        "manhattan_angle_deg": round(float(np.rad2deg(angle)), 2),
        "grid_cell_m": PX,
        "rooms": [{k: v for k, v in r.items() if not k.startswith("_")} for r in rooms],
        "adjacency": opens,
        "notes": "v0: no confidence intervals, no drift correction, no window detection yet",
    }
    (out_dir / "plan.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", type=Path)
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    K_rgb, odo, rgb_size = load_scan(args.scan)
    flip, _ = pick_convention(args.scan, odo, K_rgb, rgb_size)
    idx = np.arange(0, len(odo), args.step)
    pts = fuse(args.scan, odo, K_rgb, rgb_size, idx, flip).astype(np.float32)
    res = plan_from_points(pts, odo[:, 3], args.out / args.scan.name)

    print(f"Manhattan angle: {res['manhattan_angle_deg']} deg")
    print(f"Rooms found: {len(res['rooms'])}")
    for r in res["rooms"]:
        print(f"  R{r['id']}: {r['floor_area_m2']} m2, {r['bbox_m'][0]} x {r['bbox_m'][1]} m, "
              f"ceiling {r['ceiling_height_m']}")
    for o in res["adjacency"]:
        print(f"  opening R{o['rooms'][0]} <-> R{o['rooms'][1]}: {o['width_m']} m")
    print(f"\nWrote {args.out / args.scan.name}/plan.png, plan.json, debug_*.png")


if __name__ == "__main__":
    main()
