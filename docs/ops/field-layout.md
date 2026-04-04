# 2026 REBUILT (Welded) Field Layout

Field data loaded at runtime via `AprilTagFields.k2026RebuiltWelded`.

## Field Dimensions

- **Length:** 16.541 m (X axis, alliance wall to alliance wall)
- **Width:** 8.069 m (Y axis, scoring table to audience)

## Hub Geometry

The hub is an octagonal structure near the center of each alliance's side. Each hub has **8 AprilTags** across 4 faces (2 tags per face).

### Blue Alliance Hub

**Center:** approximately (4.63, 4.03)

| Face | Tags | X | Y range | Facing |
|------|------|---|---------|--------|
| Back (toward blue wall) | 25, 26 | 4.022 | 4.035 – 4.390 | 180° |
| Bottom (toward Y=0) | 27, 18 | 4.270 – 4.626 | 3.431 | 270° |
| Front (toward red wall) | 19, 20 | 5.229 | 3.679 – 4.035 | 0° |
| Top (toward Y=8) | 21, 24 | 4.270 – 4.626 | 4.638 | 90° |

**Back face clearance:** The back face is at x=4.022. HUB position (3.62, 4.21) places the robot center 0.4m from the face — front bumper touching the hub. Y=4.21 is centered on the back face midpoint.

### Red Alliance Hub

**Center:** approximately (11.92, 4.03)

| Face | Tags | X | Y range | Facing |
|------|------|---|---------|--------|
| Front (toward blue wall) | 9, 10 | 12.519 | 3.679 – 4.035 | 0° |
| Bottom (toward Y=0) | 8, 5 | 11.915 – 12.271 | 3.431 | 270° |
| Back (toward red wall) | 3, 4 | 11.312 | 4.035 – 4.390 | 180° |
| Top (toward Y=8) | 2, 11 | 11.915 – 12.271 | 4.638 | 90° |

## Trench / Station Tags (z=0.889m)

| Tag | X | Y | Facing | Location |
|-----|---|---|--------|----------|
| 17 | 4.663 | 0.645 | 0° | Blue bottom trench |
| 28 | 4.588 | 0.645 | 180° | Blue bottom trench |
| 22 | 4.663 | 7.425 | 0° | Blue top trench |
| 23 | 4.588 | 7.425 | 180° | Blue top trench |
| 7 | 11.953 | 0.645 | 0° | Red bottom trench |
| 6 | 11.878 | 0.645 | 180° | Red bottom trench |
| 12 | 11.953 | 7.425 | 0° | Red top trench |
| 1 | 11.878 | 7.425 | 180° | Red top trench |

## Perimeter Tags (z=0.552m)

| Tag | X | Y | Facing | Location |
|-----|---|---|--------|----------|
| 29 | 0.008 | 0.666 | 0° | Blue wall, bottom |
| 30 | 0.008 | 1.098 | 0° | Blue wall, bottom |
| 31 | 0.008 | 3.746 | 0° | Blue wall, center |
| 32 | 0.008 | 4.178 | 0° | Blue wall, center |
| 13 | 16.533 | 7.403 | 180° | Red wall, top |
| 14 | 16.533 | 6.972 | 180° | Red wall, top |
| 15 | 16.533 | 4.324 | 180° | Red wall, center |
| 16 | 16.533 | 3.892 | 180° | Red wall, center |

## Robot Preset Positions (FieldPositions.java)

All positions are blue alliance perspective. Red positions are auto-mirrored (X flipped).

| Name | X | Y | Heading | Notes |
|------|---|---|---------|-------|
| ORIGIN | 0.0 | 0.0 | 0° | Field origin |
| LEFT | 1.0 | 7.0 | 0° | Left starting position |
| RIGHT | 1.0 | 1.0 | 0° | Right starting position |
| HUB | 3.62 | 4.21 | 0° | Front bumper touching hub back face, centered on face |
