# Benchmark Capture Plan (capture day, about 3 hours)

Every requirement on page 2 of the case study, with the capture that satisfies it.

## Equipment
- iPhone **Pro** (LiDAR) with Stray Scanner and **Polycam or Magicplan** (free tier) installed
- iPhone 17 (photo and video tiers)
- Tape measure or laser measurer, notebook or `ground_truth_template.csv` open on a laptop
- 2 to 3 sheets of A4 paper, masking tape, a marker, a cup of tea or coffee (for staged damage)

## Choose the spaces
- **Multi-room set (M):** at least 3 rooms plus the connector (hallway). Use your home.
- **Damage room (D):** one furnished room, which can be one of the M rooms.

## Step 1: Stage damage in room D (15 min)
Two damage classes, on surfaces you can photograph and measure:
- **Water stain:** soak a sheet of paper in tea or coffee, let it dry, tape it flat to a wall at about 1 m height. Measure its width and height.
- **Crack:** draw a thin crack line with a marker on a sheet of paper, tape it to another wall. Measure its length.

Record both in the ground-truth sheet (surface, height of bottom edge, width, height).

## Step 2: Measure ground truth (60 min, do this FIRST or in parallel)
For every room in M and D:
- Each **wall length**, measured at about 1 m height, corner to corner
- **Ceiling height** at the room centre
- Each **door / opening width** (inside of the frame) and height
- Each **window width** and height, and its height above the floor
- **Floor area**: computed from the walls later. Note if the room is not rectangular.

Take a phone photo of each room with its walls numbered on paper. This settles any argument about which wall is "wall 2".

## Step 3: Captures (90 min)

| # | Capture | Device | Satisfies |
|---|---|---|---|
| 1 | LiDAR, whole M set, one continuous walk | Pro | multi-room, drift ablation, LiDAR gates |
| 2 | LiDAR, whole M set, **again** | Pro | repeatability (all rooms) |
| 3 | LiDAR, room D alone | Pro | damage at LiDAR tier |
| 4 | Video, whole M set + D | iPhone 17 | video tier |
| 5 | Photos, one folder per room, M + D | iPhone 17 | photo tier and photo stitch |
| 6 | Photos, one room, **second time** | iPhone 17 | photo-tier repeatability |
| 7 | Polycam/Magicplan scan of 2 rooms, export | Pro | head-to-head |

Follow `CAPTURE_PROTOCOL.md` literally for 1 to 6. That doubles as a test of the protocol.

For 7, write down the app **name and version** (App Store → app page → Version History) and export the plan or measurements (PDF, image or JSON, whichever the free tier gives).

## Step 4: Copy everything to the Mac
```
captures/home_lidar_1/   captures/home_lidar_2/   captures/roomD_lidar/
captures/home_video/     captures/home_photos/    captures/roomX_photos_2/
benchmark/app_exports/   benchmark/ground_truth.csv
```
Raw captures are large: keep them out of git (already in `.gitignore` under `data/`; add `captures/` too).
