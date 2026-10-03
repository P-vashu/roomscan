# Capture Protocol (Route 2: stock apps)

Follow this page literally. One capture = one property. Pick ONE tier per capture.

## Before you start (all tiers)
- Turn on **every light** in every room. Open curtains in daytime.
- Open all interior doors fully.
- Put **one sheet of A4 paper flat on the floor** near the middle of each room (photo and video tiers). Leave it there while capturing.
- Remove phone cases that cover any camera.
- Clean the camera lenses with a soft cloth.

## Tier 1: LiDAR (iPhone 12 Pro or newer **Pro** model only)
1. Install **Stray Scanner** (free, App Store).
2. Stand in the hallway or main connecting space. Tap **record**.
3. Hold the phone **upright at chest height**, screen facing you.
4. Walk **slowly** (about one step per second). In each room:
   - Walk along every wall. Tilt the phone down until you see the **floor line**, then up until you see the **ceiling line**.
   - Point the phone at every **door and window frame** for 2 seconds.
5. Pass through every doorway **looking through it**, not at the floor.
6. **Finish where you started**, pointing at the same view as your first second.
7. Tap **stop**. Total time: about **1 minute per room**, 5 minutes maximum.

## Tier 2: Video (any iPhone 15 or newer)
1. Settings → Camera → Record Video → **1080p at 30 fps**. Turn **Action Mode off**.
2. Open **Camera**, choose **Video**, keep the **1x** lens, hold the phone **sideways** (landscape).
3. Walk exactly as in Tier 1, steps 2 to 6. Move **half as fast** as feels natural.
4. Make sure each A4 sheet appears in the video for at least 2 seconds.

## Tier 3: Photos (any iPhone 15 or newer)
1. Settings → Camera → Formats → **Most Compatible** (JPEG).
2. Camera app, **Photo** mode, **1x** lens, landscape. Live Photo and Portrait **off**.
3. For each room, take **4 to 8 photos**:
   - One from **each corner**, aimed diagonally across the room, with floor line and ceiling line both visible.
   - At least **one photo through each doorway** showing the next room. This is how rooms are joined.
   - The A4 sheet visible in at least **2 photos**.
4. Take each room's photos together. Don't mix rooms.

## Avoid
- **Mirrors and glass:** don't stand facing a mirror or glass door closer than 1 m. Aim at it from an angle.
- **Shiny or wet floors:** keep moving. Don't stop and point at them.
- **People or pets** walking in view.
- **Fast turns.** Turn slowly, as if carrying a full cup.
- **Low light:** if a room has no working light, use the phone torch from a second phone. Write "low light" in the room's folder name.

## Hand the files over
On the Mac, make a folder `captures/<property_name>/` and put the files in it:

| Tier | What to copy | Where it goes |
|---|---|---|
| LiDAR | Files app → On My iPhone → Stray Scanner → newest folder → long-press → **Compress** → AirDrop the zip | `captures/<name>/lidar/` (unzip there) |
| Video | AirDrop the clip from Photos | `captures/<name>/video/walkthrough.mov` |
| Photos | AirDrop each room's photos | `captures/<name>/photos/01_hall/`, `02_kitchen/`, … (one folder per room) |

Then run **one command**:
```
python run.py captures/<name>
```
The pipeline detects the tier from the folder contents and writes `out/<name>/plan.png` and `plan.json`.
