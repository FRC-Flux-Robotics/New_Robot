# Game Strategy: Auto Cycling + Teleop Scoring Positions

## Problem
No competition-ready auto routine or teleop scoring workflow. Robot can drive but has no game-specific strategy for REBUILT. Need autonomous fuel cycling from neutral zone and reliable teleop scoring positions that resist defensive play.

## Priority: P0 - Critical
Competition Week 7. This is match-deciding functionality.

## Added: 2026-03-28

## Design

### Autonomous: Fuel Cycling
- **Approach:** Fixed PathPlanner paths (no vision dependency for pickup)
- **Flow:** Start position -> neutral zone fuel pickup -> return to scoring position -> score -> (repeat if time allows)
- **Target:** 1 cycle guaranteed, 2 cycles stretch goal in 15s auto period
- **Paths:** Created in PathPlanner GUI, stored in `deploy/pathplanner/paths/`
- **Alliance:** Paths defined for blue, auto-mirrored for red via PathPlanner settings

### Teleop: Preset Scoring Positions
- **Positions:** 3-4 alliance-neutral scoring spots defined in `FieldPositions.java`
  - Placeholder coordinates, tuned on the field
  - Example: close-left, close-right, far-center, safe-corner
- **Navigation:** PathPlanner pathfinding (on-the-fly path to target pose)
- **Anti-defense:** X-brake (SwerveRequest.SwerveDriveBrake) once at position
  - Simple, already implemented, no PID tuning needed
  - Driver can override at any time by moving sticks
- **Bindings:** POV/D-pad buttons on driver controller to select position

### Field Positions (Blue Alliance, to be tuned)
Positions are placeholders. Actual coordinates measured on field.

| Name | X (m) | Y (m) | Heading | Purpose |
|------|-------|-------|---------|---------|
| ScoreCloseLeft | TBD | TBD | TBD | Near scoring, left side |
| ScoreCloseRight | TBD | TBD | TBD | Near scoring, right side |
| ScoreFarCenter | TBD | TBD | TBD | Far scoring, center |
| ScoreSafe | TBD | TBD | TBD | Safe fallback, hard to pin |

## Status: Planned
