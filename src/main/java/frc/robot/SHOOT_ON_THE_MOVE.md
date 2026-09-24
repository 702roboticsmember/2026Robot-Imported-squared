# Shoot-on-the-Move Upgrade — Team 702 -

**Status:** Not implemented
**Priority:** Tier 1 — Pre-competition
**References:** Team 1939 `InterpolatingShooterSolutionFinder.java`, Team 3636 `Shooter.kt:436-462`, Team 5507 `SwerveSubsystem.java getVirtualTarget()`, Team 6328 `LaunchCalculator.java:276-288`

---

## Problem Statement

Team 702's current `AutoAimCommand` uses a physics-based ballistic derivation to compute hood angle and flywheel velocity from Limelight distance. It has three critical defects:

1. **NaN crash risk** — `CalculateVx()` calls `Math.sqrt(discriminant)` without checking if `discriminant < 0`. At unexpected distances, the robot program crashes or sends garbage to the motors.
2. **No velocity compensation** — `getVrx()` and `getVrz()` read IMU data but always return `0`. Moving-robot shot error is ignored entirely.
3. **Stub velocity input** — `CalculateVs(vx, vy, 0)` hardcodes robot velocity as zero in the final speed calculation.

The result: the robot must stop to shoot accurately. That costs ~2 seconds per shot cycle in auto and limits teleop efficiency significantly.

---

## Why Team 702's Turret Changes Everything

Most FRC teams rotate the **entire robot** to aim. Their auto-aim math produces a **drive heading**.

Team 702 has a **rotating turret independent of the drivetrain**. Auto-aim must produce:
- A **turret angle** (degrees, relative to robot heading)
- A **hood angle** (degrees)
- A **flywheel velocity** (RPS)

Of the 10 peer teams analyzed, only three solved this in a way that directly outputs turret angle. Those are the references for this upgrade.

---

## Recommended Architecture: Two-Layer System

```
Layer 1 — Shot Solution (stationary accuracy)
  InterpolatingTreeMap<distance → {flywheelRPS, hoodAngle, timeOfFlight}>
  Built from measured data at 5–8 field distances

Layer 2 — Velocity Compensation (moving accuracy)
  compensatedShotVector = idealShotVector - robotVelocity
  virtualDistance = rawDistance × (compensatedNorm / idealHorizontalSpeed)
  Look up Layer 1 at virtualDistance
```

This is Team 1939's approach. It eliminates the quadratic formula entirely — no square root, no discriminant, no NaN. Tuning is done with real measured data instead of estimated physics constants.

---

## Current Code Locations (Team 702)

| File | Relevant Section | Issue |
|------|-----------------|-------|
| `commands/AutoAimCommand.java:200-245` | `CalculateVy()`, `CalculateVx()`, `CalculateShootAngle()` | Physics formula, NaN risk, no velocity comp |
| `commands/AutoAimCommand.java:168-196` | `getVrx()`, `getVrz()` | Always returns 0 — dead code |
| `commands/AutoAimCommand.java:228-233` | `Math.sqrt(discriminant)` | Unguarded — crash on bad distance |
| `RobotContainer.java:171-178` | `AimAndShoot()` | Wraps commands in lambdas that discard them — silent no-op |

---

## Implementation Plan

### Step 1 — Create `ShooterSolutionFinder.java`

New file: `src/main/java/frc/robot/lib/shooter/ShooterSolutionFinder.java`

```java
package frc.robot.lib.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterSolutionFinder {

    // Units: distance in meters, RPS for flywheel, degrees for hood, seconds for TOF
    private static final InterpolatingDoubleTreeMap FLYWHEEL_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_MAP     = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap TOF_MAP      = new InterpolatingDoubleTreeMap();

    static {
        // FORMAT: put(distanceMeters, value)
        // Collect this data by shooting at measured distances from the hub.
        // Start with at least 5 points spanning your realistic shot range.
        // Example placeholder values — REPLACE WITH MEASURED DATA:
        //                dist    flywheel RPS
        FLYWHEEL_MAP.put(2.0,    60.0);
        FLYWHEEL_MAP.put(3.0,    70.0);
        FLYWHEEL_MAP.put(4.0,    80.0);
        FLYWHEEL_MAP.put(5.0,    88.0);
        FLYWHEEL_MAP.put(6.0,    95.0);

        //                dist    hood degrees
        HOOD_MAP.put(2.0,        50.0);
        HOOD_MAP.put(3.0,        55.0);
        HOOD_MAP.put(4.0,        59.0);
        HOOD_MAP.put(5.0,        63.0);
        HOOD_MAP.put(6.0,        65.5);

        //                dist    time of flight seconds
        TOF_MAP.put(2.0,         0.25);
        TOF_MAP.put(3.0,         0.35);
        TOF_MAP.put(4.0,         0.45);
        TOF_MAP.put(5.0,         0.55);
        TOF_MAP.put(6.0,         0.65);
    }

    public record ShooterSolution(double flywheelRPS, double hoodDegrees, double timeOfFlight) {}

    /** Returns shooter solution for a given distance with no velocity compensation (stationary shot). */
    public static ShooterSolution solve(double distanceMeters) {
        return new ShooterSolution(
            FLYWHEEL_MAP.get(distanceMeters),
            HOOD_MAP.get(distanceMeters),
            TOF_MAP.get(distanceMeters)
        );
    }

    /**
     * Returns shooter solution compensated for robot velocity (moving shot).
     *
     * Algorithm from Team 1939 InterpolatingShooterSolutionFinder:
     *   1. Compute the ideal shot direction vector to hub
     *   2. Compute ideal horizontal ball speed at that distance
     *   3. Subtract robot velocity vector from ideal shot vector (compensate)
     *   4. Scale raw distance by speed ratio to get "virtual distance"
     *   5. Look up solution at virtual distance
     *
     * @param distanceMeters      raw distance from Limelight to hub (meters)
     * @param vxMetersPerSecond   robot field-relative x velocity
     * @param vyMetersPerSecond   robot field-relative y velocity
     * @param targetAngleRad      angle from robot to hub (field-relative, radians)
     */
    public static ShooterSolution solveWithVelocity(
            double distanceMeters,
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double targetAngleRad) {

        ShooterSolution rawSolution = solve(distanceMeters);

        double rawTimeOfFlight = rawSolution.timeOfFlight();
        if (rawTimeOfFlight <= 0 || distanceMeters <= 0) {
            return rawSolution; // fallback to stationary solution
        }

        // Ideal horizontal ball speed at this raw distance
        double idealHorizontalSpeed = distanceMeters / rawTimeOfFlight;

        // Ideal shot direction unit vector
        double idealDirX = Math.cos(targetAngleRad);
        double idealDirY = Math.sin(targetAngleRad);

        // Ideal shot velocity vector
        double idealVx = idealDirX * idealHorizontalSpeed;
        double idealVy = idealDirY * idealHorizontalSpeed;

        // Compensate: subtract robot velocity from ideal shot vector
        double compVx = idealVx - vxMetersPerSecond;
        double compVy = idealVy - vyMetersPerSecond;
        double compensatedSpeed = Math.hypot(compVx, compVy);

        if (compensatedSpeed <= 0) {
            return rawSolution;
        }

        // Virtual distance: scale raw distance by speed ratio
        double virtualDistance = distanceMeters * (compensatedSpeed / idealHorizontalSpeed);

        // Clamp to measured range to avoid extrapolation crashes
        virtualDistance = Math.max(2.0, Math.min(6.0, virtualDistance));

        return solve(virtualDistance);
    }
}
```

---

### Step 2 — Create `TurretAimCalculator.java`

New file: `src/main/java/frc/robot/lib/shooter/TurretAimCalculator.java`

Handles converting a shot solution into a turret angle, accounting for robot velocity and the turret's physical offset from robot center.

```java
package frc.robot.lib.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretAimCalculator {

    // Turret offset from robot center (meters). Measure from robot center to turret pivot.
    // Positive X = forward, positive Y = left
    private static final Translation2d TURRET_OFFSET = new Translation2d(0.0, 0.0); // TODO: measure

    /**
     * Computes the required turret angle (field-relative) to hit the hub,
     * accounting for robot velocity using iterative convergence (Team 5507 / Team 6328 pattern).
     *
     * @param robotPose       current robot pose (field-relative)
     * @param robotVelocity   field-relative robot ChassisSpeeds as Translation2d(vx, vy)
     * @param hubPosition     field position of the hub
     * @return                field-relative angle the turret should point
     */
    public static Rotation2d computeTurretAngle(
            Pose2d robotPose,
            Translation2d robotVelocity,
            Translation2d hubPosition) {

        // Turret pivot position in field coordinates
        Translation2d turretPivot = robotPose.getTranslation()
            .plus(TURRET_OFFSET.rotateBy(robotPose.getRotation()));

        Translation2d targetPos = hubPosition;
        double timeOfFlight = 0.0;

        // Iterative convergence: 6 iterations (Team 5507 pattern)
        // Each pass refines the predicted ball landing point based on TOF
        for (int i = 0; i < 6; i++) {
            double dist = turretPivot.getDistance(targetPos);
            ShooterSolutionFinder.ShooterSolution sol = ShooterSolutionFinder.solve(dist);
            timeOfFlight = sol.timeOfFlight();

            // Predict where the TARGET is when the ball arrives
            // For a static hub this is just the hub — for a moving target, add target velocity here
            targetPos = hubPosition; // hub is static, no update needed
            // If target were moving: targetPos = hubPosition.plus(targetVelocity.times(timeOfFlight));
        }

        // Final angle from turret pivot to converged target position
        Translation2d toTarget = targetPos.minus(turretPivot);
        Rotation2d fieldAngle = new Rotation2d(toTarget.getX(), toTarget.getY());

        // Convert field-relative angle to robot-relative turret angle
        return fieldAngle.minus(robotPose.getRotation());
    }

    /**
     * Computes required turret angle using Team 3636's vector math approach.
     * Correct when robot IS moving — compensates shot vector for robot velocity.
     * Uses atan2 (no NaN risk).
     *
     * @param fieldAngleToHub     field-relative angle from turret to hub (Rotation2d)
     * @param distanceMeters      raw distance to hub
     * @param robotVelocity       field-relative robot velocity as Translation2d(vx, vy)
     * @param robotHeading        current robot heading
     * @return                    robot-relative turret angle to command
     */
    public static Rotation2d computeTurretAngleVectorMath(
            Rotation2d fieldAngleToHub,
            double distanceMeters,
            Translation2d robotVelocity,
            Rotation2d robotHeading) {

        ShooterSolutionFinder.ShooterSolution sol = ShooterSolutionFinder.solve(distanceMeters);
        double timeOfFlight = sol.timeOfFlight();
        if (timeOfFlight <= 0) return fieldAngleToHub.minus(robotHeading);

        double idealHorizontalSpeed = distanceMeters / timeOfFlight;

        // Ideal shot velocity vector (direction × speed)
        double idealVx = fieldAngleToHub.getCos() * idealHorizontalSpeed;
        double idealVy = fieldAngleToHub.getSin() * idealHorizontalSpeed;

        // Compensated shot vector: subtract robot velocity
        double compVx = idealVx - robotVelocity.getX();
        double compVy = idealVy - robotVelocity.getY();

        // Use atan2 — no NaN risk, correct for all quadrants (Team 3636 pattern)
        Rotation2d adjustedFieldAngle = new Rotation2d(Math.atan2(compVy, compVx));

        // Convert to robot-relative turret angle
        return adjustedFieldAngle.minus(robotHeading);
    }
}
```

---

### Step 3 — Rewrite `AutoAimCommand.java`

Replace the current physics-based execute logic. Keep the same subsystem wiring — only change what gets calculated.

```java
// In AutoAimCommand.execute():

// 1. Get distance from Limelight (existing code — keep this)
double distanceMeters = getDistanceToHub(); // your existing Limelight distance method

// 2. Get robot velocity from swerve (existing — ChassisSpeeds already available)
ChassisSpeeds fieldRelativeSpeeds = s_Swerve.getFieldRelativeSpeeds();
Translation2d robotVelocity = new Translation2d(
    fieldRelativeSpeeds.vxMetersPerSecond,
    fieldRelativeSpeeds.vyMetersPerSecond);

// 3. Get hub direction from current pose
Translation2d hubPosition = Constants.AutoAimConstants.HUB_POSITION;
Translation2d toHub = hubPosition.minus(s_Swerve.getPose().getTranslation());
double targetAngleRad = Math.atan2(toHub.getY(), toHub.getX());

// 4. Compute shot solution with velocity compensation (REPLACES all CalculateV* methods)
ShooterSolutionFinder.ShooterSolution solution = ShooterSolutionFinder.solveWithVelocity(
    distanceMeters, robotVelocity.getX(), robotVelocity.getY(), targetAngleRad);

// 5. Compute turret angle using vector math (REPLACES CalculateShootAngle)
Rotation2d turretAngle = TurretAimCalculator.computeTurretAngleVectorMath(
    new Rotation2d(targetAngleRad),
    distanceMeters,
    robotVelocity,
    s_Swerve.getHeading());

// 6. Command subsystems (same as before — only inputs change)
t_TurretSubsystem.goToAngle(turretAngle.getDegrees());
h_HoodSubsystem.goToAngle(solution.hoodDegrees());
s_ShooterSubsystem.setVelocity(solution.flywheelRPS());
```

---

### Step 4 — Add Hub Position Constant

In `Constants.java`, add to `AutoAimConstants`:

```java
public static final class AutoAimConstants {
    // Field position of the hub center (meters from blue alliance corner)
    // Measure from the 2026 field drawings or use AprilTag pose.
    // Update for actual 2026 REBUILT field layout.
    public static final Translation2d HUB_POSITION = new Translation2d(8.27, 4.10); // TODO: verify

    // Shot solution range limits (must match ShooterSolutionFinder table bounds)
    public static final double MIN_SHOT_DISTANCE = 2.0; // meters
    public static final double MAX_SHOT_DISTANCE = 6.0; // meters

    // Velocity compensation: minimum robot speed before compensation is applied
    // Below this threshold, use stationary solution (avoids noise at rest)
    public static final double MIN_VELOCITY_FOR_COMPENSATION = 0.1; // m/s
}
```

---

### Step 5 — Fix `AimAndShoot()` in `RobotContainer.java`

The current implementation silently does nothing (lambda discards returned commands).

```java
// BEFORE (broken — commands are discarded):
private Command AimAndShoot() {
    return new InstantCommand(() -> AutoAim())
        .andThen(new InstantCommand(() -> Shoot()));
}

// AFTER (correct command composition):
private Command AimAndShoot() {
    return AutoAim().andThen(Shoot());
}

// Also fix AutoAim() and Shoot() to return properly composed commands.
// AutoAim() should be a RunCommand or FunctionalCommand that runs AutoAimCommand.
// Shoot() should be a command that gates the indexer on isAtGoal() from the flywheel.
```

---

### Step 6 — Add `isAtGoal()` Gate Before Feeding

Current code feeds immediately regardless of flywheel speed. Add a readiness check modeled on Team 1939's pattern:

```java
// In ShooterSubsystem.java — add readiness Trigger:
public boolean isReadyToShoot() {
    double velocityError = Math.abs(
        shooterMotor2.getVelocity().getValueAsDouble() - targetVelocityRPS);
    return velocityError < VELOCITY_TOLERANCE_RPS; // e.g., 5.0 RPS
}

// In RobotContainer — gate indexer on flywheel readiness:
private Command Shoot() {
    return Commands.waitUntil(s_ShooterSubsystem::isReadyToShoot)
        .andThen(Commands.runOnce(() -> i_IndexerSubsystem.setSpeedPrimary(1.0)))
        .withTimeout(2.0); // safety timeout
}
```

---

## Data Collection Procedure

Before the lookup table is useful, you need measured data. Collect this at the first practice session.

### Setup
1. Mark distances on the floor: 2m, 2.5m, 3m, 3.5m, 4m, 4.5m, 5m, 5.5m, 6m from hub center
2. Use the Limelight distance reading (not tape measure) — you want the distances the code will see in competition
3. Log flywheel RPS, hood angle, and shot time to SmartDashboard during collection

### For each distance:
1. Position robot at marked distance, **stationary**
2. Manually set flywheel speed and hood angle via SmartDashboard overrides
3. Shoot 5 balls — adjust until 4/5 are consistent hub shots
4. Record: `distanceMeters`, `flywheelRPS`, `hoodDegrees`
5. Time 3 shots with a stopwatch → average = `timeOfFlight`
6. Enter all values into `ShooterSolutionFinder.java` static block

### Minimum viable table (5 points)
You need at least 5 distances to get useful interpolation. 8 points gives significantly better accuracy, especially at the edges of your shooting range.

### Verify interpolation
After entering data, test at distances between your measured points (e.g., 2.7m if you measured at 2.5 and 3.0). The interpolated values should produce accurate shots without re-tuning.

---

## Testing Checklist

### Stationary accuracy (no velocity compensation)
- [ ] Shot consistent at 2m
- [ ] Shot consistent at 3m
- [ ] Shot consistent at 4m
- [ ] Shot consistent at 5m
- [ ] Shot consistent at intermediate distances (interpolation working)

### Moving accuracy (velocity compensation active)
- [ ] Driving toward hub at 1 m/s → shot still accurate
- [ ] Driving away from hub at 1 m/s → shot still accurate
- [ ] Strafing at 1 m/s → shot still accurate
- [ ] Driving toward hub at 2+ m/s → confirm convergence loop handles it

### Safety
- [ ] At distance < 2m (min table bound) → no crash, fallback to stationary solution
- [ ] At distance > 6m (max table bound) → no crash, clamped to max
- [ ] With vision lost (distance = 0 or NaN) → command exits or holds last known setpoints

### Integration
- [ ] `AimAndShoot()` actually fires the indexer (was previously a silent no-op)
- [ ] `isReadyToShoot()` gate prevents feeding before flywheel is at speed
- [ ] Turret follows compensated angle while driving
- [ ] Hood reaches target angle before ball is fed

---

## Phased Rollout

| Phase | What | When |
|-------|------|------|
| **1 — NaN fix** | Guard discriminant in current `CalculateVx()` with `if (discriminant < 0) return 0` | Immediate — 15 min |
| **2 — Table foundation** | Implement `ShooterSolutionFinder` with placeholder data, replace physics formula in `AutoAimCommand` | Before first practice |
| **3 — Data collection** | Fill table with real measured data at practice session | First practice |
| **4 — Velocity comp** | Enable `solveWithVelocity()` — add robot ChassisSpeeds input | After stationary accuracy confirmed |
| **5 — Convergence loop** | Add `TurretAimCalculator` iterative convergence for moving-target turret angle | After velocity comp validated |

---

## Source References

| Team | File | Lines | Pattern Used |
|------|------|-------|-------------|
| Team 1939 | `InterpolatingShooterSolutionFinder.java` | 81–122 | Interpolating table + velocity compensation |
| Team 3636 | `Shooter.kt` | 436–462 | Vector math with `atan2` / `acos` (no NaN) |
| Team 5507 | `SwerveSubsystem.java` | `getVirtualTarget()` | 6-iteration convergence loop |
| Team 6328 | `LaunchCalculator.java` | 276–288 | 20-iteration lookahead, offset launcher geometry |
| Team 6328 | `LaunchCalculator.java` | 349–363 | `asin(offset/distance)` turret pivot correction |

Local copies at: `/Users/brad/github/FRC/otherteams/`
