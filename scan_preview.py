#!/usr/bin/env python3
"""Load a Stray Scanner capture, fuse LiDAR depth into one point cloud,
and write a top-down preview image plus rough stats.

Usage:
    python scan_preview.py data/raw/single_room            # every 10th frame
    python scan_preview.py data/raw/single_room --step 5

Outputs go to out/<scan_name>/:
    topdown.png   wall-band density seen from above (1 px = 1 cm)
    cloud.ply     fused point cloud (open in MeshLab / CloudCompare)
    stats.json    frame counts, pose convention, rough floor/ceiling height
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

DEPTH_SCALE = 1000.0          # Stray Scanner depth PNGs are uint16 millimetres
MIN_DEPTH, MAX_DEPTH = 0.15, 5.0
# Flip from OpenCV camera axes (y down, z forward) to ARKit camera axes (y up, z back)
CV_TO_ARKIT = np.diag([1.0, -1.0, -1.0])


def load_scan(folder: Path):
    K_rgb = np.loadtxt(folder / "camera_matrix.csv", delimiter=",")
    # timestamp, frame, x, y, z, qx, qy, qz, qw  (trailing columns ignored)
    odo = np.genfromtxt(folder / "odometry.csv", delimiter=",",
                        skip_header=1, usecols=range(9))
    cap = cv2.VideoCapture(str(folder / "rgb.mp4"))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1440
    cap.release()
    return K_rgb, odo, (w, h)


def depth_intrinsics(K_rgb, rgb_size, depth_shape):
    """Scale RGB intrinsics down to the depth map resolution."""
    sx = depth_shape[1] / rgb_size[0]
    sy = depth_shape[0] / rgb_size[1]
    K = K_rgb.copy()
    K[0, 0] *= sx
    K[1, 1] *= sy
    K[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
    K[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
    return K


def backproject(depth_m, conf, K, min_conf):
    h, w = depth_m.shape
    v, u = np.indices((h, w))
    m = (depth_m > MIN_DEPTH) & (depth_m < MAX_DEPTH)
    if conf is not None:
        m &= conf >= min_conf
    z = depth_m[m]
    x = (u[m] - K[0, 2]) * z / K[0, 0]
    y = (v[m] - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1)  # OpenCV camera frame


def pose_matrix(row):
    _, _, x, y, z, qx, qy, qz, qw = row
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T  # camera -> world


def fuse(folder, odo, K_rgb, rgb_size, frame_idx, arkit_flip, min_conf=2):
    pts, K = [], None
    for i in frame_idx:
        fid = int(odo[i, 1])
        dpath = folder / "depth" / f"{fid:06d}.png"
        if not dpath.exists():
            continue
        depth = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED).astype(np.float32) / DEPTH_SCALE
        cpath = folder / "confidence" / f"{fid:06d}.png"
        conf = cv2.imread(str(cpath), cv2.IMREAD_UNCHANGED) if cpath.exists() else None
        if K is None:
            K = depth_intrinsics(K_rgb, rgb_size, depth.shape)
        p_cam = backproject(depth, conf, K, min_conf)
        if arkit_flip:
            p_cam = p_cam @ CV_TO_ARKIT.T
        T = pose_matrix(odo[i])
        pts.append(p_cam @ T[:3, :3].T + T[:3, 3])
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def voxel_count(pts, size=0.05):
    return len(np.unique(np.floor(pts / size).astype(np.int64), axis=0))


def pick_convention(folder, odo, K_rgb, rgb_size):
    """The correct camera-axis convention gives a sharper (fewer-voxel) cloud."""
    idx = np.linspace(0, len(odo) - 1, 40).astype(int)
    scores = {}
    for flip in (True, False):
        pts = fuse(folder, odo, K_rgb, rgb_size, idx, flip)
        scores[flip] = voxel_count(pts) / max(len(pts), 1)
    return min(scores, key=scores.get), scores


def floor_ceiling(pts):
    """Rough floor/ceiling from peaks in the world-Y (gravity-up) histogram."""
    y = pts[:, 1]
    lo, hi = np.percentile(y, [0.05, 99.95])
    bins = np.arange(lo, hi + 0.01, 0.01)
    hist, edges = np.histogram(y, bins=bins)
    centers = (edges[:-1] + edges[1:]) / 2
    n = len(hist)
    f = int(np.argmax(hist[: max(n // 3, 1)]))
    c = n - max(n // 3, 1) + int(np.argmax(hist[n - max(n // 3, 1):]))
    floor_share = hist[f] / hist.sum()
    ceil_share = hist[c] / hist.sum()
    return float(centers[f]), float(centers[c]), float(floor_share), float(ceil_share)


def topdown(pts, floor_y, ceil_y, path, px=0.01):
    band = pts[(pts[:, 1] > floor_y + 0.3) & (pts[:, 1] < ceil_y - 0.3)]
    if len(band) == 0:
        band = pts
    xz = band[:, [0, 2]]
    mn = xz.min(0) - 0.2
    ij = np.floor((xz - mn) / px).astype(int)
    W, H = ij.max(0) + 1
    img = np.zeros((H, W), np.float32)
    np.add.at(img, (ij[:, 1], ij[:, 0]), 1)
    img = np.log1p(img)
    img = (255 * img / max(img.max(), 1e-6)).astype(np.uint8)
    img = cv2.cvtColor(255 - img, cv2.COLOR_GRAY2BGR)
    bar = int(1.0 / px)  # 1 m scale bar
    cv2.line(img, (10, H - 10), (10 + bar, H - 10), (0, 0, 255), 3)
    cv2.putText(img, "1 m", (10, H - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.imwrite(str(path), img)
    return float((W * px)), float((H * px))


def write_ply(pts, path, max_pts=500_000):
    if len(pts) > max_pts:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_pts, replace=False)]
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n")
    with open(path, "wb") as fh:
        fh.write(header.encode())
        fh.write(pts.astype("<f4").tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan", type=Path)
    ap.add_argument("--step", type=int, default=10, help="use every Nth frame")
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    K_rgb, odo, rgb_size = load_scan(args.scan)
    flip, scores = pick_convention(args.scan, odo, K_rgb, rgb_size)
    idx = np.arange(0, len(odo), args.step)
    pts = fuse(args.scan, odo, K_rgb, rgb_size, idx, flip)
    floor_y, ceil_y, fs, cs = floor_ceiling(pts)

    out = args.out / args.scan.name
    out.mkdir(parents=True, exist_ok=True)
    w, d = topdown(pts, floor_y, ceil_y, out / "topdown.png")
    write_ply(pts, out / "cloud.ply")

    duration = float(odo[-1, 0] - odo[0, 0])
    stats = {
        "scan": args.scan.name,
        "frames": int(len(odo)),
        "duration_s": round(duration, 1),
        "fps": round(len(odo) / max(duration, 1e-6), 1),
        "rgb_size": rgb_size,
        "frames_fused": int(len(idx)),
        "points": int(len(pts)),
        "arkit_axis_flip": bool(flip),
        "convention_scores": {str(k): round(v, 4) for k, v in scores.items()},
        "floor_y_m": round(floor_y, 3),
        "ceiling_y_m": round(ceil_y, 3),
        "rough_ceiling_height_m": round(ceil_y - floor_y, 3),
        "floor_peak_share": round(fs, 3),
        "ceiling_peak_share": round(cs, 3),
        "topdown_extent_m": [round(w, 2), round(d, 2)],
    }
    (out / "stats.json").write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))
    print(f"\nWrote {out}/topdown.png, cloud.ply, stats.json")


if __name__ == "__main__":
    main()
