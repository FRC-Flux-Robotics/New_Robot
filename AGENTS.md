# AGENTS.md — `New_Robot` Repository Guide (FRC Team 10413)

> **Standalone Repository Guide:** `New_Robot` is the 2026 season (`FUEL`) competition codebase for FRC Team 10413 (FLUX Robotics) running on roboRIO 2.0 + CTRE Phoenix 6 swerve + PhotonVision (`edu.wpi.first.*`, WPILib 2026, JDK 17).

## 1. Key Architecture & Operational Docs
- [`README.md`](README.md): Repository overview, subsystem layout (`SwerveDrive`, `Vision`, `Intake`, `Tilter`, `Indexer`, `Feeder`, `Shooter`, `Hood`), and controls.
- [`docs/ops/reference.md`](docs/ops/reference.md): Pre-match and pit operational reference guide.
- [`docs/ops/camera-calibration.md`](docs/ops/camera-calibration.md) & [`docs/ops/vision-calibration.md`](docs/ops/vision-calibration.md): Camera extrinsics and PhotonVision / AprilTag calibration procedures.
- [`docs/ops/fuel-test-protocol.md`](docs/ops/fuel-test-protocol.md): `FUEL` shooter range-table and mechanism test protocol.
- [`docs/ops/field-layout.md`](docs/ops/field-layout.md) & [`docs/ops/quick-card.md`](docs/ops/quick-card.md): Field dimensions and pit quick-reference card.

## 2. Build & Test Commands
Run unit tests and build from the repository root using JDK 17:
```bash
./gradlew test build
```

## 3. Core Engineering Rules
- **NWU Coordinate System:** WPILib uses North-West-Up (`+X` forward, `+Y` left, `+Z` up, `+CCW` rotation) with `Wall/Blue` (`WPIBlue`) origin `(0,0)` at the right corner of the Blue alliance wall.
- **CTRE Phoenix 6 Current Limits:** Always configure both `StatorCurrentLimit` and `SupplyCurrentLimit` on every `TalonFX`, and ensure swerve `SlipCurrent` does not exceed `60 A` (since `withSlipCurrent` overwrites `StatorCurrentLimit` on drive motors).
- **Zero Per-Loop Allocations:** Reuse `private final` CTRE `ControlRequest` fields inside `periodic()` and `execute()`.
