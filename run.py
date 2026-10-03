#!/usr/bin/env python3
"""One command per capture:  python run.py captures/<name>  [--step N]

Detects the tier from what is in the folder:
  LiDAR  : a Stray Scanner folder (odometry.csv + depth/) anywhere inside
  Video  : a .mov / .mp4 file and no depth data
  Photos : sub-folders of .jpg / .jpeg / .heic images (one per room)
"""
import argparse
import sys
from pathlib import Path

import numpy as np

IMG_EXT = {".jpg", ".jpeg", ".heic", ".png"}
VID_EXT = {".mov", ".mp4"}


def find_lidar(root: Path):
    for p in [root, *sorted(root.rglob("*"))]:
        if p.is_dir() and (p / "odometry.csv").exists() and (p / "depth").is_dir():
            return p
    return None


def detect(root: Path):
    lidar = find_lidar(root)
    if lidar:
        return "lidar", lidar
    vids = [p for p in root.rglob("*") if p.suffix.lower() in VID_EXT]
    if vids:
        return "video", vids[0]
    rooms = [d for d in sorted(root.rglob("*")) if d.is_dir()
             and any(f.suffix.lower() in IMG_EXT for f in d.iterdir())]
    if rooms:
        return "photo", rooms
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    if not args.capture.exists():
        sys.exit(f"Not found: {args.capture}")
    tier, src = detect(args.capture)
    out_dir = args.out / args.capture.name
    print(f"Capture: {args.capture}  ->  tier: {tier}")

    if tier == "lidar":
        from scan_preview import load_scan, pick_convention, fuse
        from floorplan import plan_from_points
        K_rgb, odo, rgb_size = load_scan(src)
        flip, _ = pick_convention(src, odo, K_rgb, rgb_size)
        idx = np.arange(0, len(odo), args.step)
        pts = fuse(src, odo, K_rgb, rgb_size, idx, flip).astype(np.float32)
        res = plan_from_points(pts, odo[:, 3], out_dir)
        print(f"Rooms: {len(res['rooms'])}, openings: {len(res['adjacency'])}")
    elif tier == "video":
        sys.exit("Video tier not implemented yet.")
    elif tier == "photo":
        sys.exit(f"Photo tier not implemented yet ({len(src)} room folders found).")
    else:
        sys.exit("No LiDAR folder, video or photo folders found. See CAPTURE_PROTOCOL.md.")
    print(f"Wrote {out_dir}/plan.png and plan.json")


if __name__ == "__main__":
    main()
