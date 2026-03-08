# Autonomous Routines - Beginner Guide

This guide explains how to create autonomous routines for the FLUX robot. Autonomous code runs at the start of each match for 15 seconds with no driver input.

---

## Key Concepts

### Coordinate System (WPILib NWU)

The field uses a coordinate system where:
- **+X** = forward (away from your driver station)
- **+Y** = left (from the driver's perspective)
- **Rotation** = counter-clockwise is positive

```
        +X (forward)
         ^
         |
  +Y <---+--- -Y
         |
        -X (backward)
```

All distances are in **meters**. All angles are in **radians** (use `Math.toRadians(degrees)` to convert).

### Commands

Every autonomous action is a `Command`. Commands can be:
- **Single actions** — drive to a point, follow a path, wait
- **Composed together** — run one after another, run in parallel, race

### DriveInterface

All driving commands come from `DriveInterface`. Your auto methods receive it as a parameter:

```java
public static Command myAuto(DriveInterface drive) {
    // use drive.followPath(), drive.driveToPose(), etc.
}
```

---

## Step-by-Step: Your First Auto

### 1. Create a Method in Autos.java

Open `src/main/java/frc/robot/Autos.java` and add a new `public static` method:

```java
public static Command myFirstAuto(DriveInterface drive) {
    // your code here
}
```

### 2. Build a Path

Use `PathPlannerPath` to define where the robot should go:

```java
PathPlannerPath path = new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
                List.of(
                        new Pose2d(0, 0, Rotation2d.kZero),      // start here
                        new Pose2d(2, 0, Rotation2d.kZero))),     // end 2m forward
        new PathConstraints(2.0, 1.5, Math.PI, Math.PI),          // speed limits
        null,                                                      // ideal starting state (null = use current)
        new GoalEndState(0, Rotation2d.kZero));                   // end stopped, facing forward
```

**Breaking it down:**

| Part | What It Does |
|------|-------------|
| `waypointsFromPoses(List.of(...))` | List of points the robot drives through |
| `Pose2d(x, y, rotation)` | A position on the field (meters) with a facing direction |
| `PathConstraints(maxVel, maxAccel, maxAngVel, maxAngAccel)` | How fast the robot is allowed to go |
| `GoalEndState(endVelocity, endRotation)` | How fast and which direction at the end (0 = stop) |

### 3. Follow the Path

```java
return drive.followPath(path);
```

### 4. Register in RobotContainer

Open `src/main/java/frc/robot/RobotContainer.java` and add your auto to the chooser:

```java
autoChooser.addOption("My First Auto", Autos.myFirstAuto(drivetrain));
```

Now it appears on SmartDashboard for the driver to select.

### 5. Test It

```bash
# Build to check for errors
./gradlew build

# Run in simulation
./gradlew simulateJava
```

---

## Common Patterns

### Drive Forward (Simplest Auto)

```java
public static Command driveForward(DriveInterface drive) {
    PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                    List.of(
                            new Pose2d(0, 0, Rotation2d.kZero),
                            new Pose2d(2, 0, Rotation2d.kZero))),
            new PathConstraints(2.0, 1.5, Math.PI, Math.PI),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    return drive.followPath(path);
}
```

### Chain Multiple Paths (Sequential)

Use `.andThen()` to run one path after another:

```java
public static Command forwardTurnBack(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI);

    // Leg 1: drive 2m forward
    PathPlannerPath forward = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                    List.of(
                            new Pose2d(0, 0, Rotation2d.kZero),
                            new Pose2d(2, 0, Rotation2d.kZero))),
            constraints,
            null,
            new GoalEndState(0, Rotation2d.kZero));

    // Leg 2: turn around, drive 1m back
    PathPlannerPath back = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                    List.of(
                            new Pose2d(2, 0, Rotation2d.k180deg),
                            new Pose2d(1, 0, Rotation2d.k180deg))),
            constraints,
            null,
            new GoalEndState(0, Rotation2d.k180deg));

    return drive.followPath(forward)
            .andThen(drive.followPath(back));
}
```

### Drive to a Specific Pose (Short Range, PID)

For short moves (<2m) where you need precision, use `driveToPose`:

```java
public static Command driveToScorePosition(DriveInterface drive) {
    Pose2d target = new Pose2d(1.5, 0.5, Rotation2d.fromDegrees(45));
    return drive.driveToPose(target, 0.05); // 5cm tolerance
}
```

### Pathfind to a Pose (Long Range, Obstacle-Aware)

For longer moves where PathPlanner generates the path at runtime:

```java
public static Command navigateToFarSide(DriveInterface drive) {
    Pose2d target = new Pose2d(10, 4, Rotation2d.kZero);
    return drive.pathfindToPose(target);
}
```

### Add a Pause Between Actions

```java
import edu.wpi.first.wpilibj2.command.Commands;

// Wait 1 second between paths
return drive.followPath(path1)
        .andThen(Commands.waitSeconds(1.0))
        .andThen(drive.followPath(path2));
```

### Add a Timeout (Safety)

Prevent an auto from running forever if something goes wrong:

```java
return drive.followPath(path).withTimeout(5.0); // give up after 5 seconds
```

---

## Command Composition Cheat Sheet

| Method | What It Does |
|--------|-------------|
| `a.andThen(b)` | Run A, then run B |
| `a.alongWith(b)` | Run A and B at the same time, finish when both done |
| `a.raceWith(b)` | Run A and B at the same time, finish when first one done |
| `a.withTimeout(5.0)` | Run A, but give up after 5 seconds |
| `a.until(() -> condition)` | Run A until condition is true |
| `Commands.waitSeconds(1.0)` | Do nothing for 1 second |
| `Commands.none()` | Do nothing (useful as a placeholder) |

---

## Common Rotations

Instead of calculating radians, use these shortcuts:

| Code | Angle |
|------|-------|
| `Rotation2d.kZero` | 0 degrees (facing forward) |
| `Rotation2d.k90deg` | 90 degrees (facing left) |
| `Rotation2d.k180deg` | 180 degrees (facing backward) |
| `Rotation2d.fromDegrees(45)` | Any angle in degrees |
| `Rotation2d.fromRadians(Math.PI / 4)` | Any angle in radians |

---

## PathConstraints Explained

```java
new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration)
```

| Parameter | Unit | Recommended Start | Description |
|-----------|------|-------------------|-------------|
| maxVelocity | m/s | 2.0 | Top speed along the path |
| maxAcceleration | m/s^2 | 1.5 | How fast it speeds up/slows down |
| maxAngularVelocity | rad/s | Math.PI | Top turning speed |
| maxAngularAcceleration | rad/s^2 | Math.PI | How fast turning speeds up |

**Tips:**
- Start slow (2.0 m/s) and increase after testing
- The robot's absolute max is ~5 m/s, but 3.0-3.5 is safer for auto
- Lower acceleration = smoother but slower, less wheel slip

---

## GoalEndState Explained

```java
new GoalEndState(endVelocity, endRotation)
```

| Parameter | Meaning |
|-----------|---------|
| `endVelocity` | Speed at end of path (0 = stop, >0 = keep moving into next path) |
| `endRotation` | Which direction the robot faces when it finishes |

Use `endVelocity > 0` when chaining paths that should flow smoothly without stopping.

---

## Required Imports

When writing auto routines, you'll typically need:

```java
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivetrain.DriveInterface;
import java.util.List;
```

---

## Checklist for New Autos

- [ ] Method is `public static Command` in `Autos.java`
- [ ] Takes `DriveInterface drive` as parameter
- [ ] Path starts at a reasonable pose (usually where the robot begins)
- [ ] PathConstraints are conservative (start slow, speed up later)
- [ ] GoalEndState velocity is 0 unless chaining into another path
- [ ] Registered in `RobotContainer` auto chooser
- [ ] Builds without errors (`./gradlew build`)
- [ ] Tested in simulation (`./gradlew simulateJava`)
- [ ] Has a timeout for safety (`.withTimeout(seconds)`)

---

*Team 10413 FLUX Robotics - 2026 Season*
