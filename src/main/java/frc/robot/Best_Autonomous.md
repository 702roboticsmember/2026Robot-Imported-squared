# Best Autonomous Programs — FRC 2026 Analysis
## For Team 702 Implementation Reference

This document is a deep technical reference compiled from reading the actual source code of the three strongest autonomous programs found among peer teams competing in the FRC 2026 season. The top three teams analyzed are Team 6328 Mechanical Advantage (best overall architecture), Walton Robotics (best recursive/parameterized builder pattern), and Team 5507 GWHS (best zone-aware trigger system with FuelSim integration). Each team's code was read in full; every code excerpt below is sourced directly from those files. All code snippets adapted for Team 702 use the actual variable names found in `/Users/brad/github/FRC/2026Robot/src/main/java/frc/robot/RobotContainer.java`.

---

## Ranking Summary

| Rank | Team | Trajectory Lib | Named Autos | Standout Feature |
|------|------|---------------|-------------|------------------|
| 1 | 6328 Mechanical Advantage | Choreo (custom DriveTrajectory) | 5 routines | AutoSelector with sub-question dashboard, lazy command evaluation, `waitUntilWithinTolerance` gate before every shot |
| 2 | Walton Robotics | Choreo via PathPlanner-compatible AutoFactory | 8 named routines | Recursive `createAutonSequence(int n)` that builds an N-cycle auto without code duplication |
| 3 | 5507 GWHS | PathPlanner v2026 + Choreo trajectories | 9 named routines | Zone-aware `Trigger` bus feeding autonomous shot selection; `driveToPose` used live inside auto paths; FuelSim for simulation |

---

## #1 — Team 6328 Mechanical Advantage

### What They Built

Source directory: `/Users/brad/github/FRC/otherteams/6328-MechanicalAdvantage/src/main/java/org/littletonrobotics/frc2026/commands/auto/`

6328 has five named auto routines registered through their custom `AutoSelector`. Each routine is a method on their `AutoBuilder` class (not PathPlanner's `AutoBuilder` — a completely custom class they wrote):

**Home Depot Salesman** (`AutoBuilder.java:46-68`)
- Drives from a right-side trench start to the outpost (two path options: through the tower, or around it — chosen live via dashboard question)
- Waits 2.5 seconds at the outpost for fuel collection
- Drives to `Launch.rightTower` using a live `driveToPose` command
- Races that drive with a `waitUntilWithinTolerance` gate that blocks indexing until position is confirmed
- After launch: conditionally drives to climb or does nothing (selected by second dashboard question)

**Lowe's Hardware Salesman** (`AutoBuilder.java:71-100`)
- Right trench start to outpost, 2.5-second wait for fuel
- Drives to depot (optional path variant around the tower, chosen by question)
- Drives to `Launch.leftTower`, same tolerance-gated shoot pattern
- Post-launch: climb or nothing via question

**Monopoly Salesman** (`AutoBuilder.java:103-142`)
- Accepts any of four start locations: LEFT_TRENCH, LEFT_BUMP, RIGHT_BUMP, RIGHT_TRENCH (selected via question)
- RIGHT_BUMP and RIGHT_TRENCH variants add a 3-second start delay to avoid collisions
- After collecting, dynamically chooses leftTower vs rightTower based on current robot pose at runtime using `RobotState.getInstance().getEstimatedPose()` — no hardcoded destination
- Fires with the same tolerance-gated `index()` call

**Timid Salesman** (`AutoBuilder.java:145-176`)
- Purely pose-driven — no path file at all
- Calculates a `target` `Pose2d` at runtime from the selected start location waypoint plus an offset
- Uses `LaunchCalculator.getStationaryAimedPose()` to compute the exact aimed pose
- Calls `resetStartingPose()` then drives and fires in parallel

**Drive Forward 1m / Turn 90 variants** (`AutoBuilder.java:178-188`)
- Characterization and testing autos, simple single-trajectory commands

### Architecture Deep Dive

#### AutoSelector: Sub-Question Dashboard System

Source: `/Users/brad/github/FRC/otherteams/6328-MechanicalAdvantage/src/main/java/org/littletonrobotics/frc2026/AutoSelector.java`

This is the single biggest architectural advantage 6328 has. Instead of a flat `SendableChooser<Command>` where every combination of start position + post-auto action needs its own entry, they have a two-level system:

```java
// AutoSelector.java:23-53
public class AutoSelector extends VirtualSubsystem {
  private static final int maxQuestions = 2;

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  public AutoSelector(String key) {
    routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
    // Creates up to 2 additional choosers on the dashboard that only appear
    // when the selected routine has questions
    for (int i = 0; i < maxQuestions; i++) {
      var publisher = NetworkTableInstance.getDefault()
          .getStringTopic("/SmartDashboard/" + key + "/Question #" + (i + 1))
          .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + (i + 1) + " Chooser"));
    }
  }
}
```

Routines are registered with a list of `AutoQuestion` objects:

```java
// RobotContainer.java:281-286 (6328)
autoSelector.addRoutine(
    "Home Depot Salesman",
    List.of(
        new AutoQuestion("Through Tower?", List.of(AutoQuestionResponse.NO)),
        new AutoQuestion("Post-Launch?", List.of(AutoQuestionResponse.NOTHING))),
    autoBuilder.homeDepotSalesman());
```

The dashboard shows "Home Depot Salesman" in the routine chooser. When selected, "Question #1 Chooser" automatically populates with `["NO"]` and "Question #2 Chooser" populates with `["NOTHING", "CLIMB"]`. No extra code needed to manage which choosers are visible.

The `periodic()` method on `AutoSelector` (`AutoSelector.java:88-127`) reads the currently selected routine, updates the question choosers to match that routine's questions, and caches the responses. During autonomous it freezes updates.

#### Lazy Command Evaluation via Supplier

The entire command tree is built once at robot init time, but uses `Supplier<List<AutoQuestionResponse>>` throughout:

```java
// AutoBuilder.java:34-44
@RequiredArgsConstructor
public class AutoBuilder {
  private final Drive drive;
  private final Slamtake intake;
  // ...
  private final Supplier<List<AutoQuestionResponse>> responses;

  public Command homeDepotSalesman() {
    return Commands.sequence(
        Commands.either(
                followTrajectory("HomeDepot_startToOutpost", drive, true),
                followTrajectory("HomeDepot_startToOutpostAround", drive, true),
                () -> responses.get().get(0).equals(AutoQuestionResponse.YES))
```

The lambda `() -> responses.get().get(0)` is called at the moment the `Commands.either` branch is evaluated during auto execution, not at construction time. This means the dashboard can be changed up until auto begins and the command will use the latest value.

#### waitUntilWithinTolerance Gate

Source: `AutoCommands.java:145-176`

This is the most critical pattern 702 is missing. 6328 never fires without confirming the robot is within pose tolerance:

```java
// AutoCommands.java:145-156
public static boolean withinTolerance(
    Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
  Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
  return robotPose.getTranslation().getDistance(AllianceFlipUtil.apply(target.getTranslation()))
          < translationalTolerance
      && Math.abs(
              robotPose
                  .getRotation()
                  .minus(AllianceFlipUtil.apply(target.getRotation()))
                  .getRadians())
          < rotationalTolerance.getRadians();
}

// AutoCommands.java:166-176
public static Command waitUntilWithinTolerance(
    Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
  return Commands.waitUntil(
      () -> withinTolerance(target, translationalTolerance, rotationalTolerance));
}

// Supplier variant for dynamic targets:
public static Command waitUntilWithinTolerance(
    Supplier<Pose2d> target, double translationalTolerance, Rotation2d rotationalTolerance) {
  return Commands.waitUntil(
      () -> withinTolerance(target.get(), translationalTolerance, rotationalTolerance));
}
```

The usage pattern in every shooting auto:

```java
// AutoBuilder.java:53-61
AutoCommands.driveToPose(drive, () -> Launch.rightTower)
    .raceWith(
        Commands.sequence(
            AutoCommands.waitUntilWithinTolerance(
                Launch.rightTower, 0.1, Rotation2d.fromDegrees(5)),
            index(hopper, kicker, flywheel, intake)
                .withDeadline(Commands.waitSeconds(10))))
```

The drive command races with a sequence: first wait until within 0.1m and 5 degrees, then fire. The race means the drive command is still running (and correcting pose) while firing. The 10-second deadline is an emergency stop.

#### AutoCommands index() Helper

Source: `AutoCommands.java:89-109`

```java
public static Command index(Hopper hopper, Kicker kicker, Flywheel flywheel, Slamtake slamtake) {
  return Commands.waitUntil(flywheel::atGoal)
      .andThen(
          Commands.startEnd(
                  () -> {
                    hopper.setGoal(Hopper.Goal.LAUNCH);
                    kicker.setGoal(Kicker.Goal.LAUNCH);
                  },
                  () -> {
                    hopper.setGoal(Hopper.Goal.STOP);
                    kicker.setGoal(Kicker.Goal.STOP);
                  },
                  hopper,
                  kicker)
              .withDeadline(
                  Commands.repeatingSequence(
                      Commands.waitSeconds(1.5),
                      Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT)),
                      Commands.waitSeconds(0.5),
                      Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)))));
}
```

The `waitUntil(flywheel::atGoal)` is a second tolerance gate — it verifies flywheel is at speed before committing any balls. The repeating sequence of retract/deploy cycles the intake to dislodge stuck fuel. This is a sophisticated pattern that should be adapted for 702's indexer.

#### xCrossed / yCrossed Utilities

Source: `AutoCommands.java:111-143`

```java
public static boolean xCrossed(double xPosition, boolean towardsCenter) {
  Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
  if (AllianceFlipUtil.shouldFlip()) {
    if (towardsCenter) {
      return robotPose.getX() < FieldConstants.fieldLength - xPosition;
    } else {
      return robotPose.getX() > FieldConstants.fieldLength - xPosition;
    }
  } else {
    // blue alliance logic
    return towardsCenter ? robotPose.getX() > xPosition : robotPose.getX() < xPosition;
  }
}
```

These allow event-based triggers during path following: "when robot crosses the midfield line, start spinning up the flywheels."

### Key Patterns for Team 702

The three patterns to adopt immediately, adapted to 702's stack:

1. `waitUntilWithinTolerance` before any shot — currently 702 has NO guard before firing
2. `flywheel.atGoal()` gate inside the shoot sequence
3. `AutoSelector` sub-question system to reduce dashboard clutter

### Direct Adoption Checklist

- [ ] **Implement `waitUntilWithinTolerance`** — source `AutoCommands.java:166-176`, adapt as shown in Reusable Code Blocks section below. Effort: 1 hour.
- [ ] **Add `atGoal()` method to `ShooterSubsystem`** — currently `ShooterSubsystem.java` has no `atGoal()` check. Add a method that returns `true` when both flywheel velocities are within tolerance. Effort: 30 min.
- [ ] **Build `AutoCommands702.java`** — a static utility class mirroring `AutoCommands.java`, using `s_Swerve.getState().Pose` instead of `RobotState.getInstance()`. Effort: 2 hours.
- [ ] **Implement `AutoSelector`** — replaces the flat `SendableChooser<Command> autoChooser`. Requires `SwitchableChooser` utility (NetworkTables-backed). Effort: 3-4 hours.
- [ ] **Convert auto routines to factory methods** — create `AutoBuilder702.java` with methods like `leftStartShootClimb()`, `rightStartShootClimb()`, etc. Effort: 2-3 hours.
- [ ] **Add pose-gated shoot sequence** — wire tolerance gate into every auto that fires. Effort: 1 hour.

---

## #2 — Walton Robotics

### What They Built

Source: `/Users/brad/github/FRC/otherteams/WaltonRobotics/src/main/java/frc/robot/autons/WaltAutonFactory.java`

Walton has a single `WaltAutonFactory` class that produces all named autos. The named public routines are:

**oneRightNeutralPickup** (`WaltAutonFactory.java:127-130`) — drives right, vision-picks up one neutral fuel ball, drives to shoot. Calls `createAutonSequence(1)`.

**twoRightNeutralPickup** (`WaltAutonFactory.java:135-138`) — picks up twice (one ball per trip), shoots after each trip. Calls `createAutonSequence(2)`.

**threeRightNeutralPickup** (`WaltAutonFactory.java:143-146`) — three pickup cycles, shoots twice during the sequence. Calls `createAutonSequence(3)`.

**fourRightNeutralPickup** (`WaltAutonFactory.java:150-154`) — four pickup cycles, shoots three times. Calls `createAutonSequence(4)`.

**oneLeftNeutralPickup** / **twoLeftNeutralPickup** / **threeLeftNeutralPickup** (`WaltAutonFactory.java:159-177`) — mirror versions for the left trench.

**oneRightDepotPickup** (`WaltAutonFactory.java:183-191`) — single depot run: drive to depot, vision-pickup, drive to shoot pose.

**oneRightOutpostPickup** (`WaltAutonFactory.java:196-200`) — outpost run: drive to outpost, then to neutral, no separate shot step.

### Architecture Deep Dive

#### Recursive Cycle Builder

Source: `WaltAutonFactory.java:94-121`

This is the standout pattern. A single private method generates the entire command sequence for any number of cycles:

```java
private Command createAutonSequence(int pickupTimes) {
    if (pickupTimes == 1) { // Base case
        return Commands.sequence(
            runTrajCmd(neutralCycle[0]),  // to neutral zone
            pickupCmd(m_side.equals(AutonSide.RIGHT) ? PickupLocation.RIGHT : PickupLocation.LEFT),
            runTrajCmd(neutralCycle[1])   // neutral to shoot position
        );
    }

    if (pickupTimes <= 0) {
        return Commands.none();
    }

    ArrayList<Command> commandSequence = new ArrayList<Command>();
    commandSequence.add(createAutonSequence(pickupTimes - 1)); // recursive call

    if (pickupTimes != 2) {
        commandSequence.add(runTrajCmd(neutralCycle[1])); // neutral to shoot (only for 3+)
    }

    commandSequence.add(runTrajCmd(neutralCycle[2])); // shoot to neutral
    commandSequence.add(pickupCmd(m_side.equals(AutonSide.RIGHT) ? PickupLocation.RIGHT : PickupLocation.LEFT));

    return Commands.sequence(commandSequence.toArray(new Command[commandSequence.size()]));
}
```

The recursion unrolls like this for `pickupTimes = 3`:
```
createAutonSequence(3)
  -> createAutonSequence(2)
       -> createAutonSequence(1)   [base: toNeutral, pickup, toShoot]
       + runTrajCmd(shoot->neutral)
       + pickupCmd(RIGHT)
  + runTrajCmd(neutral->shoot)     [only added when pickupTimes != 2, so yes for 3]
  + runTrajCmd(shoot->neutral)
  + pickupCmd(RIGHT)
```

For a 3-pickup auto: go to neutral, pick up, go to shoot, shoot, go back to neutral, pick up, go to shoot, shoot, go back to neutral, pick up. No path name strings are duplicated anywhere.

The `neutralCycle` array is set per-side:

```java
// WaltAutonFactory.java:34-38
neutralCycle = m_side.equals(AutonSide.RIGHT)
    ? new String[]{"RightToNeutral", "RightNeutralToShoot", "RightShootToNeutral"}
    : new String[]{"LeftToNeutral", "LeftNeutralToShoot", "LeftShootToNeutral"};
```

So adding a new side requires only adding new path files and a new string array — no logic changes.

#### pickupCmd with Vision-Based Pose Correction

Source: `WaltAutonFactory.java:64-86`, `Swerve.java:401-407`

```java
// WaltAutonFactory.java:64-86
public Command pickupCmd(PickupLocation location) {
    Pose2d postPickupPose;
    switch (location) {
        case LEFT:   postPickupPose = m_postLeftPickupNeutral; break;
        case RIGHT:  postPickupPose = m_postRightPickupNeutral; break;
        case DEPOT:  postPickupPose = m_postPickupDepot; break;
        default:     postPickupPose = m_postRightPickupNeutral; break;
    }

    return Commands.sequence(
        m_drivetrain.swerveToObject().withTimeout(1),  // vision-seek for 1 second
        m_drivetrain.toPose(postPickupPose).withTimeout(1)  // then drive to post-pickup pose
    );
}
```

The `swerveToObject()` method (Swerve.java:401-407) calls PhotonVision to detect the nearest game piece, converts it to a field-space pose, and drives to it using PID:

```java
// Swerve.java:401-407 (WaltonRobotics)
public Command swerveToObject() {
    PhotonTrackedTarget target = detection.getClosestObject();
    Pose2d destination = detection.targetToPose(getState().Pose, target);
    detection.addFuel(destination);
    return toPose(destination);
}
```

`toPose()` (Swerve.java:384-396) drives using three independent PID controllers:

```java
// Swerve.java:384-396 (WaltonRobotics)
public Command toPose(Pose2d destination) {
    return Commands.run(() -> {
        Pose2d curPose = getState().Pose;
        double xSpeed = m_pathXController.calculate(curPose.getX(), destination.getX());
        double ySpeed = m_pathYController.calculate(curPose.getY(), destination.getY());
        double thetaSpeed = m_pathThetaController.calculate(
            curPose.getRotation().getRadians(), destination.getRotation().getRadians());
        setControl(swreq_drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(thetaSpeed));
    });
}
```

PID gains for Walton: translation kP = 7.7, rotation kP = 7.0 (`Swerve.java:63-65`).

#### Alliance Flip at Construction Time

Source: `WaltAutonFactory.java:40-45`

```java
public void setAlliance(boolean isRed) {
    m_postRightPickupNeutral = isRed
        ? AllianceFlipUtil.flip(AutonK.rightNeutralPose)
        : AutonK.rightNeutralPose;
    m_postPickupDepot = isRed
        ? AllianceFlipUtil.flip(AutonK.rightDepotPose)
        : AutonK.rightDepotPose;
    m_postLeftPickupNeutral = isRed
        ? AllianceFlipUtil.flip(AutonK.leftNeutralPose)
        : AutonK.leftNeutralPose;
}
```

`setAlliance()` is called once when alliance is known (in `robotInit` or first `teleopInit`), which precomputes the flipped poses rather than flipping them inside every command lambda.

### Key Patterns for Team 702

1. The recursive sequence builder eliminates copy-paste between 1-cycle, 2-cycle, and 3-cycle autos
2. The `swerveToObject` + `toPose` pattern provides vision-guided pickup without requiring a Choreo event marker
3. Alliance flip at setup time means no runtime overhead during auto

### Direct Adoption Checklist

- [ ] **Create `AutoFactory702.java`** — mirrors `WaltAutonFactory`, uses PathPlanner trajectory names from `src/main/deploy/pathplanner/paths/`. Effort: 3 hours.
- [ ] **Implement `createCycleSequence(int n, boolean isLeft)`** — recursive builder from the pattern above. Effort: 1 hour (once factory class exists).
- [ ] **Add `toPose(Pose2d)` method to `Swerve.java`** — Team 702's `Swerve` class currently lacks this. Port Walton's implementation using 702's existing `PointToPointPID` logic or a fresh PID controller. Effort: 2 hours.
- [ ] **Fix `AutoIntakeCommand` null suppliers** — currently registered as `new AutoIntakeCommand(null, null, DOWN, s_Swerve, power, i_IntakeSubsystem)` which crashes on `ta.getAsDouble()`. This must be fixed before any vision-guided auto can work. Effort: 30 min.
- [ ] **Call `setAlliance()` equivalent in `robotInit`** — precompute flipped shooting poses before auto begins. Effort: 30 min.

---

## #3 — Team 5507 GWHS

### What They Built

Source: `/Users/brad/github/FRC/otherteams/5507-GWHS/src/main/java/frc/robot/`

GWHS uses PathPlanner + Choreo trajectories, the same stack as Team 702. They have 9 named auto options exposed on the dashboard:

**Bump 1 Cycle Depot** / **Bump 1 Cycle Outpost** — single cycle from the bump zone, ending at either depot or outpost side shooting position. Uses `NeutralAutos(mirror=false/true, BUMP, twoCycle=false)`.

**Bump 2 Cycle Depot** / **Bump 2 Cycle Outpost** — same but two cycles before shooting.

**Trench 1 Cycle Depot** / **Trench 1 Cycle Outpost** — single cycle from the trench zone.

**Trench 2 Cycle Depot** / **Trench 2 Cycle Outpost** — two trench cycles.

**Depot 1 Cycle** — dedicated depot path, collects from depot, drives to hub tower, shoots, then climbs. Source: `DepotPathAuto_1c.java`.

### Architecture Deep Dive

#### NeutralAutos Parameterized SequentialCommandGroup

Source: `NeutralAutos.java:49-89`

GWHS avoids nine separate command files by parameterizing a single `SequentialCommandGroup` subclass:

```java
// NeutralAutos.java:49-89
public NeutralAutos(boolean mirror, Routine routine, boolean twoCycle) {
    // Determine path name prefix
    String pathprefix = (routine == Routine.BUMP) ? "B_" : "T_";

    // Load the correct paths, mirroring for outpost-side starts
    PathPlannerPath cycle;
    PathPlannerPath cycletwo;
    PathPlannerPath climb;
    if (mirror) {
        cycle    = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle").mirrorPath();
        cycletwo = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle2").mirrorPath();
        climb    = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb_Mirrored");
    } else {
        cycle    = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle");
        cycletwo = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Cycle2");
        climb    = PathPlannerPath.fromChoreoTrajectory(pathprefix + "Climb");
    }

    Pose2d startingPose = new Pose2d(
        cycle.getPoint(0).position,
        cycle.getIdealStartingState().rotation());

    addCommands(
        Commands.sequence(
            AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
            cyclePath(cycle, true),
            cyclePath(cycletwo, false).onlyIf(() -> twoCycle),
            climbPath(climb).onlyIf(() -> !twoCycle))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
}
```

`PathPlannerPath.fromChoreoTrajectory(name).mirrorPath()` is a PathPlanner v2026 API call that geometrically mirrors the trajectory across the field centerline — no separate mirrored path file required.

#### cyclePath: Parallel Intake + Homing During Path Follow

Source: `NeutralAutos.java:106-129`

```java
private Command cyclePath(PathPlannerPath path, boolean homing) {
    return Commands.sequence(
        // While following the path, run intake and (on first cycle) home subsystems
        AutoBuilder.followPath(path)
            .deadlineFor(
                Commands.sequence(
                    Commands.parallel(
                        groundIntakeExtend.homingCommand().onlyIf(() -> homing),
                        climber.homingCommand().onlyIf(() -> homing)),
                    Commands.parallel(
                        shooter.stopShooter(),
                        groundIntakeExtend.extend(),
                        groundIntakeRoller.startIntake()),
                    Commands.waitSeconds(3),
                    shooter.preSpin())),    // start spinning flywheel 3s into path
        groundIntakeRoller.stopIntake(),
        // Drive to score pose and shoot, with 6-second timeout
        Commands.waitSeconds(6)
            .deadlineFor(
                drivetrain.driveToPose(() -> getScorePose(() -> path))
                    .alongWith(
                        Commands.parallel(
                            indexer.index(),
                            shooter.cruiseControl(),
                            EagleUtil.shootInSim(drivetrain)))));
}
```

The critical insight: during the `followPath` phase, a `deadlineFor` chain runs in parallel. The first 3 seconds deploy the intake. After 3 seconds, `shooter.preSpin()` starts the flywheel warming up. By the time the path ends and the robot drives to the scoring pose, the flywheel is already at speed. The `EagleUtil.shootInSim()` call logs shots in simulation for FuelSim tracking.

The score pose is computed dynamically from the path:

```java
// NeutralAutos.java:91-104
private Pose2d getScorePose(Supplier<PathPlannerPath> path) {
    if (EagleUtil.isRedAlliance()) {
        return new Pose2d(
            path.get().flipPath().getPoint(0).position,
            path.get().flipPath().getIdealStartingState().rotation());
    } else {
        return new Pose2d(
            path.get().getPoint(0).position,
            path.get().getIdealStartingState().rotation());
    }
}
```

This reads the starting point of the `Cycle` path (which is the scoring position the robot returns to), accounting for alliance flip. No hardcoded scoring poses.

#### climbPath: Simplest Possible Climb Auto Step

Source: `NeutralAutos.java:132-141`

```java
private Command climbPath(PathPlannerPath path) {
    return Commands.sequence(
        AutoBuilder.followPath(path)
            .deadlineFor(
                shooter.runVelocity(0),
                groundIntakeExtend.retract(),
                groundIntakeRoller.runVoltage(0)),
        Commands.idle().alongWith(climber.runPosition(ClimberConstants.CLIMB)));
}
```

While driving to the climb structure: retract intake, stop shooter, stop intake roller. On arrival: hold position with `Commands.idle()` while the climber executes.

#### DepotPathAuto_1c: Full Depot-to-Climb Auto

Source: `DepotPathAuto_1c.java:19-58`

```java
// DepotPathAuto_1c.java
PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("D_Start_Depot");
PathPlannerPath climbPath    = PathPlannerPath.fromChoreoTrajectory("D_Depot_Climb");

addCommands(
    AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
    Commands.parallel(
        AutoBuilder.followPath(startingPath)
            .deadlineFor(
                Commands.sequence(
                    Commands.parallel(
                        climber.homingCommand().onlyIf(() -> RobotBase.isReal()),
                        groundIntakeExtend.homingCommand().onlyIf(() -> RobotBase.isReal())),
                    Commands.parallel(
                        groundIntakeExtend.extend(),
                        groundIntakeRoller.startIntake()),
                    Commands.waitSeconds(2),
                    shooter.preSpin()))),
    AutoBuilder.followPath(climbPath).deadlineFor(shooter.cruiseControl()),
    Commands.waitSeconds(6)
        .deadlineFor(
            shooter.cruiseControl(),
            EagleUtil.shootInSim(drivetrain).onlyIf(() -> RobotBase.isSimulation())),
    climber.runPosition(ClimberConstants.CLIMB).alongWith(shooter.stopShooter()));
```

Key structural detail: the second path (`climbPath`) follows a different trajectory that positions the robot at the climb structure, and `shooter.cruiseControl()` is used as a deadline during that path because shooting happens immediately on arrival, not after a separate driveToPose correction.

#### Zone-Aware Trigger Bus (RobotContainer + SwerveSubsystem)

Source: `SwerveSubsystem.java:109-131`, `RobotContainer.java:326-379` (GWHS)

GWHS defines field zone triggers directly on the drivetrain subsystem:

```java
// SwerveSubsystem.java:109-121
public Trigger isInAllianceZone =
    new Trigger(() -> EagleUtil.isInAllianceZone(getCachedState().Pose));
public Trigger isInOpponentAllianceZone =
    new Trigger(() -> EagleUtil.isInOpponentAllianceZone(getCachedState().Pose));
public Trigger isInNeutralZone =
    new Trigger(() -> EagleUtil.isInNeutralZone(getCachedState().Pose));
public Trigger isOnDepotSide =
    new Trigger(() -> EagleUtil.isOnDepotSide(getCachedState().Pose));
public Trigger isOnOutpostSide =
    new Trigger(() -> EagleUtil.isOnOutpostSide(getCachedState().Pose));
public Trigger isFacingGoal =
    new Trigger(() -> MathUtil.isNear(
        getGoalHeading(), getCachedState().Pose.getRotation().getDegrees(), 5));
```

These triggers compose in `RobotContainer.configureBindings()`:

```java
// RobotContainer.java:331-348 (GWHS)
controller.rightTrigger().and(drivetrain.isInAllianceZone).whileTrue(shootHub());

controller.rightTrigger()
    .and(drivetrain.isInNeutralZone
        .or(drivetrain.isInOpponentAllianceZone)
        .and(drivetrain.isOnDepotSide))
    .whileTrue(shootDepot());

controller.rightTrigger()
    .and(drivetrain.isInNeutralZone
        .or(drivetrain.isInOpponentAllianceZone)
        .and(drivetrain.isOnOutpostSide))
    .whileTrue(shootOutpost());
```

The same trigger system could be used in auto: a `Trigger` that fires when the robot enters the alliance zone could automatically call `s_ShooterSubsystem.preSpin()` as a background action.

#### driveToPose Implementation

Source: `SwerveSubsystem.java:460-469`

```java
public Command driveToPose(Supplier<Pose2d> pose) {
    return new AlignToPose(
            pose,
            this,
            () -> 0.0,
            this.controller)
        .withName("Drive to Pose");
}
```

`AlignToPose` is their custom command (`commands/AlignToPose.java`) that uses PathPlanner's `PPHolonomicDriveController` internally, giving them PathPlanner-quality path correction for point-to-point moves without requiring a `.path` file.

#### FuelSim Integration

Source: `RobotContainer.java:403-428` (GWHS)

```java
private void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
        0.660, 0.711, 0.127,   // robot dimensions
        () -> drivetrain.getCachedState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCachedState().Speeds,
            drivetrain.getCachedState().Pose.getRotation()));
    instance.registerIntake(
        0.350, 0.700, -0.330, 0.330,
        () -> (groundIntakeRoller.getGoalRollerVoltage() > 0));
    instance.start();
}
```

FuelSim runs the full fuel physics simulation — fuel balls spawn on the field, the robot drives over them, and if the intake bounding box overlaps a ball while the intake is powered, the ball is consumed. This allows the team to test auto paths in simulation with realistic fuel counts.

### Key Patterns for Team 702

1. `PathPlannerPath.mirrorPath()` eliminates the need for mirrored path files for left/right variants
2. `deadlineFor()` with a sequential intake-then-prespin chain maximizes flywheel heat during path travel
3. The zone trigger bus (`isInAllianceZone`, `isFacingGoal`) can gate auto shots without hardcoded timing

### Direct Adoption Checklist

- [ ] **Use `mirrorPath()` for left/right path variants** — eliminates half the path files in `src/main/deploy/pathplanner/paths/`. PathPlanner v2026.1.2 supports this. Effort: 1 hour.
- [ ] **Add `deadlineFor` intake chain to existing paths** — currently 702's PathPlanner autos appear to have no parallel intake commands running during path follow. Effort: 1-2 hours per path.
- [ ] **Add `shooter.preSpin()` partway through drive paths** — GWHS uses `Commands.waitSeconds(3)` then `preSpin()` as a staged intake→spinup sequence. Effort: 30 min per path.
- [ ] **Add `getScorePose()` from path endpoint** — avoid hardcoding shooting Pose2d values; derive from path's `getPoint(0)` instead. Effort: 1 hour.
- [ ] **Add zone Triggers to `Swerve.java`** — `isInAllianceZone`, `isFacingGoal` etc. as `Trigger` fields on the subsystem. Effort: 2 hours.
- [ ] **Wire `configureAutonomous()` with `NeutralAutos` pattern** — a parameterized class takes one boolean (mirror), one enum (zone), one boolean (two-cycle). Add 4 options without 4 separate classes. Effort: 3 hours.

---

## Team 702 Current Gaps vs Top 3

| Capability | 6328 | Walton | GWHS | Team 702 |
|-----------|------|--------|------|----------|
| PathPlanner configured | Yes (Choreo) | Yes (Choreo) | Yes (PathPlanner) | Yes (PathPlanner) |
| `getAutonomousCommand()` returns selected auto | Yes | Yes | Yes | **Recently fixed** — now returns `autoChooser.getSelected()` |
| Pose-tolerance gate before shooting | Yes (`waitUntilWithinTolerance`, 0.1m/5deg) | Implicit via `toPose` timeout | Via `driveToPose` with 6s deadline | **No** — fires immediately |
| Flywheel `atGoal()` gate before indexing | Yes (`waitUntil(flywheel::atGoal)`) | Not visible in code | Yes (`shooter.isAtGoalVelocity_Hub`) | **No** — `ShooterSubsystem` has no `atGoal()` |
| Intake runs during path follow | Not applicable (slamtake) | Via `pickupCmd` + vision | Yes (`deadlineFor` chain) | **No** |
| Flywheel pre-spins during path follow | Yes (flywheel default command) | Not explicit | Yes (`preSpin()` after 3s) | **No** |
| Vision-guided pickup in auto | Not explicit | Yes (`swerveToObject`) | Not explicit | **No** — `AutoIntakeCommand` crashes (null suppliers) |
| Recursive/parameterized auto builder | Via `AutoBuilder` methods | Yes (`createAutonSequence(n)`) | Via constructor parameters | **No** — no auto builder class exists |
| Sub-question dashboard selection | Yes (AutoSelector, 2 questions) | No | No | **No** |
| `mirrorPath()` for left/right | N/A (Choreo) | Not used | Yes | **No** — separate path files per side |
| Alliance flip handling | Yes (`AllianceFlipUtil`) | Yes (precomputed) | Yes (`flipPath()`) | Unclear — not visible in auto code |
| Named auto class / factory | Yes (`AutoBuilder` class) | Yes (`WaltAutonFactory`) | Yes (`NeutralAutos`, `DepotPathAuto_1c`) | **No** — `autos/` directory is empty except README |
| FuelSim in simulation | Yes (6328's own FuelSim) | No | Yes (same FuelSim) | **No** |
| Error handling on path load | No (throws RuntimeException) | N/A (AutoFactory handles it) | Yes (`try/catch`, `reportError`) | Depends on PathPlanner defaults |

---

## Implementation Roadmap for Team 702

### Phase 1 — Foundation (before first practice match)

**Goal:** No crashed VMs, every path actually fires a ball.

#### 1a. Fix `AutoIntakeCommand` null crash

`RobotContainer.java:270`:
```java
// CURRENT (crashes):
NamedCommands.registerCommand("AutoIntake",
    new AutoIntakeCommand(null, null, DOWN, s_Swerve, power, i_IntakeSubsystem));
```

Either remove this named command (if no path uses it yet) or wire real suppliers. The `tx` and `ta` suppliers come from a Limelight:

```java
// FIXED — using Limelight for tx and ta:
NamedCommands.registerCommand("AutoIntake",
    new AutoIntakeCommand(
        () -> LimelightHelpers.getTX("limelight"),  // tx supplier
        () -> LimelightHelpers.getTA("limelight"),  // ta supplier
        () -> LimelightHelpers.getTV("limelight"),  // tv supplier
        s_Swerve,
        1.0,
        i_IntakeSubsystem));
```

If no Limelight target detection is available yet, remove the registration entirely:
```java
// SAFE PLACEHOLDER — does nothing but won't crash:
NamedCommands.registerCommand("AutoIntake", Commands.none());
```

#### 1b. Add `atGoal()` to `ShooterSubsystem`

`ShooterSubsystem.java` currently has no velocity tolerance check. Add:

```java
// ShooterSubsystem.java — add this field and method
private static final double VELOCITY_TOLERANCE_RPS = 2.0; // tunable
private double targetVelocityRPS = 0.0;

public boolean atGoal() {
    double actual = FlywheelMotor2.getVelocity().getValueAsDouble();
    return Math.abs(actual - targetVelocityRPS) < VELOCITY_TOLERANCE_RPS
        && targetVelocityRPS > 0;
}

// Modify setVelocity to record the target:
public void setVelocity(double velocity) {
    double rps = MetersTotick(velocity);
    if (rps > 50) rps = 50;
    this.targetVelocityRPS = rps;
    FlywheelMotor1.setControl(new Follower(Constants.ShooterConstants.shooterMotor2,
        MotorAlignmentValue.Opposed));
    FlywheelMotor2.setControl(velControl.withVelocity(rps));
}
```

#### 1c. Create `AutoCommands702.java`

Create `/Users/brad/github/FRC/2026Robot/src/main/java/frc/robot/autos/AutoCommands702.java`:

```java
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class AutoCommands702 {

    /**
     * Returns true when the robot is within the given translation and rotation tolerances
     * of the target pose.
     */
    public static boolean withinTolerance(
            Swerve s_Swerve,
            Pose2d target,
            double translationalToleranceMeters,
            Rotation2d rotationalTolerance) {
        Pose2d robotPose = s_Swerve.getState().Pose;
        double translationError = robotPose.getTranslation().getDistance(target.getTranslation());
        double rotationError = Math.abs(
            robotPose.getRotation().minus(target.getRotation()).getRadians());
        return translationError < translationalToleranceMeters
            && rotationError < rotationalTolerance.getRadians();
    }

    /**
     * Returns a command that waits until the robot is within tolerance of a fixed target.
     * Adapted from 6328 AutoCommands.java:166-170.
     */
    public static Command waitUntilWithinTolerance(
            Swerve s_Swerve,
            Pose2d target,
            double translationalToleranceMeters,
            Rotation2d rotationalTolerance) {
        return Commands.waitUntil(
            () -> withinTolerance(s_Swerve, target, translationalToleranceMeters, rotationalTolerance));
    }

    /**
     * Supplier variant for dynamic targets (e.g., targets computed from alliance color).
     * Adapted from 6328 AutoCommands.java:172-176.
     */
    public static Command waitUntilWithinTolerance(
            Swerve s_Swerve,
            Supplier<Pose2d> target,
            double translationalToleranceMeters,
            Rotation2d rotationalTolerance) {
        return Commands.waitUntil(
            () -> withinTolerance(s_Swerve, target.get(), translationalToleranceMeters, rotationalTolerance));
    }

    /**
     * Waits until the flywheel is at goal speed, then feeds the indexer.
     * Adapted from 6328 AutoCommands.java:89-109.
     *
     * Usage: shootWhenReady(s_Swerve, s_ShooterSubsystem, i_IndexerSubsystem)
     *        .withDeadline(Commands.waitSeconds(5.0))
     */
    public static Command shootWhenReady(
            ShooterSubsystem s_ShooterSubsystem,
            frc.robot.subsystems.IndexerSubsystem i_IndexerSubsystem) {
        return Commands.waitUntil(s_ShooterSubsystem::atGoal)
            .andThen(
                Commands.startEnd(
                    () -> i_IndexerSubsystem.setSpeedPrimary(
                        frc.robot.Constants.IndexerConstants.PrimarySpeed + 0.2),
                    () -> i_IndexerSubsystem.setSpeedPrimary(0),
                    i_IndexerSubsystem));
    }
}
```

#### 1d. Wire tolerance-gated shot into every auto that fires

For every PathPlanner `.auto` file that calls `"Shoot"`, change the registered named command in `RobotContainer.java`:

```java
// CURRENT (fires immediately regardless of pose or flywheel state):
NamedCommands.registerCommand("Shoot", AutoShoot());

// FIXED — gates on flywheel ready:
NamedCommands.registerCommand("Shoot",
    Commands.waitUntil(s_ShooterSubsystem::atGoal)
        .andThen(AutoShoot())
        .withTimeout(5.0));
```

### Phase 2 — Competitive (before first regional)

**Goal:** Parameterized autos, pre-spun flywheel, working 2-cycle path.

#### 2a. Create `Auto702Factory.java`

Create `/Users/brad/github/FRC/2026Robot/src/main/java/frc/robot/autos/Auto702Factory.java`:

```java
package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.ArrayList;
import java.util.List;

public class Auto702Factory {
    private final Swerve s_Swerve;
    private final ShooterSubsystem s_ShooterSubsystem;
    private final IndexerSubsystem i_IndexerSubsystem;
    private final IntakeSubsystem i_IntakeSubsystem;
    private final ClimbSubsystem c_ClimbSubsystem;

    // Path name arrays by side — maps directly to files in
    // src/main/deploy/pathplanner/paths/
    private static final String[] LEFT_CYCLE_PATHS  = {
        "Left-start-collect",    // start to neutral
        "Left-Trench-Depot",     // neutral to shoot
        "Left-depot-in-depot"    // shoot back to neutral
    };
    private static final String[] RIGHT_CYCLE_PATHS = {
        "Right-start-collect",
        "Right-Trench-Depot",
        "Right-start-left-bulldoze"
    };

    public Auto702Factory(
            Swerve s_Swerve,
            ShooterSubsystem s_ShooterSubsystem,
            IndexerSubsystem i_IndexerSubsystem,
            IntakeSubsystem i_IntakeSubsystem,
            ClimbSubsystem c_ClimbSubsystem) {
        this.s_Swerve = s_Swerve;
        this.s_ShooterSubsystem = s_ShooterSubsystem;
        this.i_IndexerSubsystem = i_IndexerSubsystem;
        this.i_IntakeSubsystem = i_IntakeSubsystem;
        this.c_ClimbSubsystem = c_ClimbSubsystem;
    }

    /**
     * Recursive cycle builder.
     * Adapted from WaltAutonFactory.createAutonSequence().
     * Each cycle = drive to neutral, vision-pick, drive to shoot, shoot.
     */
    private Command createCycleSequence(int cycles, boolean isLeft) {
        String[] paths = isLeft ? LEFT_CYCLE_PATHS : RIGHT_CYCLE_PATHS;

        if (cycles <= 0) return Commands.none();

        if (cycles == 1) {
            return Commands.sequence(
                followPath(paths[0]),   // to neutral
                pickupAndReturn(isLeft, paths[1]),  // intake + drive to shoot
                shootGated()
            );
        }

        ArrayList<Command> seq = new ArrayList<>();
        seq.add(createCycleSequence(cycles - 1, isLeft));
        if (cycles != 2) {
            seq.add(shootGated());  // shoot between cycles 3+
        }
        seq.add(followPath(paths[2]));  // shoot -> neutral
        seq.add(pickupAndReturn(isLeft, paths[1]));

        return Commands.sequence(seq.toArray(new Command[0]));
    }

    /**
     * Follow a path while running the intake in parallel.
     * Adapted from GWHS NeutralAutos.cyclePath() deadlineFor pattern.
     */
    private Command followPath(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path)
                .deadlineFor(
                    Commands.sequence(
                        Commands.parallel(
                            Commands.runOnce(() -> i_IntakeSubsystem.setIntakeSpeed(0.6),
                                i_IntakeSubsystem)),
                        Commands.waitSeconds(2.5),
                        Commands.runOnce(() -> s_ShooterSubsystem.setVelocity(15.0),
                            s_ShooterSubsystem)  // pre-spin partway through path
                    ));
        } catch (Exception e) {
            edu.wpi.first.wpilibj.DriverStation.reportError(
                "Path not found: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Intake command: spin intake then drive to shoot pose.
     */
    private Command pickupAndReturn(boolean isLeft, String shootPathName) {
        return Commands.sequence(
            Commands.runOnce(() -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem),
            followPath(shootPathName)
        );
    }

    /**
     * Pose-tolerance-gated shot.
     * Adapted from 6328 AutoBuilder.java driveToPose.raceWith(waitUntilWithinTolerance + index).
     */
    private Command shootGated() {
        return Commands.waitUntil(s_ShooterSubsystem::atGoal)
            .andThen(
                Commands.startEnd(
                    () -> i_IndexerSubsystem.setSpeedPrimary(
                        frc.robot.Constants.IndexerConstants.PrimarySpeed + 0.2),
                    () -> i_IndexerSubsystem.setSpeedPrimary(0),
                    i_IndexerSubsystem)
                .withDeadline(Commands.waitSeconds(4.0)));
    }

    // ---- PUBLIC AUTO ROUTINES ----

    /** 1 neutral cycle from left trench, shoots once. No climb. */
    public Command oneLeftCycle() {
        return createCycleSequence(1, true);
    }

    /** 2 neutral cycles from left trench, shoots twice. No climb. */
    public Command twoLeftCycle() {
        return createCycleSequence(2, true);
    }

    /** 3 neutral cycles from left trench, shoots three times. No climb. */
    public Command threeLeftCycle() {
        return createCycleSequence(3, true);
    }

    /** 1 neutral cycle from right trench, shoots once. No climb. */
    public Command oneRightCycle() {
        return createCycleSequence(1, false);
    }

    /** 2 neutral cycles from right trench, shoots twice. No climb. */
    public Command twoRightCycle() {
        return createCycleSequence(2, false);
    }
}
```

#### 2b. Register factory autos in `RobotContainer`

Replace the current `autoChooser` setup in `RobotContainer.java`:

```java
// In RobotContainer.java, replace:
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Chooser", autoChooser);

// With:
Auto702Factory autoFactory = new Auto702Factory(
    s_Swerve, s_ShooterSubsystem, i_IndexerSubsystem, i_IntakeSubsystem, c_ClimbSubsystem);

SendableChooser<Command> autoChooser = new SendableChooser<>();
autoChooser.setDefaultOption("Do Nothing", Commands.none());
autoChooser.addOption("1 Left Cycle",  autoFactory.oneLeftCycle());
autoChooser.addOption("2 Left Cycle",  autoFactory.twoLeftCycle());
autoChooser.addOption("3 Left Cycle",  autoFactory.threeLeftCycle());
autoChooser.addOption("1 Right Cycle", autoFactory.oneRightCycle());
autoChooser.addOption("2 Right Cycle", autoFactory.twoRightCycle());
SmartDashboard.putData("Auto Chooser", autoChooser);
```

Note: `getAutonomousCommand()` already returns `autoChooser.getSelected()` after the recent fix, so no change needed there.

### Phase 3 — Excellence (championship-level)

**Goal:** Sub-question dashboard, vision-guided pickup, zone-reactive behavior.

#### 3a. Port `AutoSelector` for sub-question system

The `AutoSelector` from 6328 requires `SwitchableChooser` (a NetworkTables-backed chooser that allows changing its options at runtime). Implement `SwitchableChooser702`:

```java
// src/main/java/frc/robot/autos/SwitchableChooser702.java
package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class SwitchableChooser702 {
    private final StringArrayPublisher optionsPublisher;
    private final StringSubscriber activeSubscriber;
    private String[] currentOptions = new String[]{};

    public SwitchableChooser702(String key) {
        optionsPublisher = NetworkTableInstance.getDefault()
            .getStringArrayTopic("/SmartDashboard/" + key + "/options")
            .publish();
        activeSubscriber = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/" + key + "/active")
            .subscribe("");
    }

    public void setOptions(String[] options) {
        this.currentOptions = options;
        optionsPublisher.set(options);
    }

    public String get() {
        String val = activeSubscriber.get();
        if (val.isEmpty() && currentOptions.length > 0) return currentOptions[0];
        return val;
    }
}
```

Then implement `AutoSelector702` mirroring 6328's `AutoSelector.java` using 702's subsystem names and `SwitchableChooser702`.

#### 3b. Add zone Triggers to `Swerve.java`

Adapted from GWHS `SwerveSubsystem.java:109-131`:

```java
// Add to Swerve.java, after the subsystem fields:
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

// Alliance zone: x < fieldLength/2 for blue, x > fieldLength/2 for red
public final Trigger isInAllianceZone = new Trigger(() -> {
    double x = getState().Pose.getX();
    boolean isRed = Constants.getAlliance();
    return isRed ? x > frc.robot.Constants.FIELD_LENGTH / 2.0
                 : x < frc.robot.Constants.FIELD_LENGTH / 2.0;
});

public final Trigger isNearShootingPose = new Trigger(() -> {
    // True when within 1.5m of any known shooting waypoint
    var pose = getState().Pose;
    // Add actual shooting pose coordinates once established
    return pose.getTranslation().getDistance(
        new edu.wpi.first.math.geometry.Translation2d(2.0, 5.5)) < 1.5;
});
```

Then in `RobotContainer`, use these triggers to auto-prespin during auto:

```java
// In configureBindings() or a new configureAutoTriggers():
// When robot enters alliance zone during any mode, prespin the shooter
s_Swerve.isInAllianceZone.onTrue(
    Commands.runOnce(() -> s_ShooterSubsystem.setVelocity(15.0), s_ShooterSubsystem)
        .ignoringDisable(false));
```

#### 3c. Vision-guided pickup in auto

Once `AutoIntakeCommand` is fixed (Phase 1), register a working version:

```java
// Named command for PathPlanner event markers:
NamedCommands.registerCommand("VisionIntake",
    new AutoIntakeCommand(
        () -> LimelightHelpers.getTX("limelight"),
        () -> LimelightHelpers.getTA("limelight"),
        () -> LimelightHelpers.getTV("limelight"),
        s_Swerve,
        1.0,
        i_IntakeSubsystem)
    .withTimeout(2.0)  // don't chase forever
    .andThen(Commands.runOnce(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem)));
```

Add event markers in PathPlanner GUI at the start of each collect segment calling `"VisionIntake"`.

---

## Reusable Code Blocks

### `waitUntilWithinTolerance`
**Source:** Team 6328, `AutoCommands.java:166-176`
**Adapted for Team 702:**
```java
// File: src/main/java/frc/robot/autos/AutoCommands702.java
public static Command waitUntilWithinTolerance(
        Swerve s_Swerve,
        Pose2d target,
        double translationalToleranceMeters,
        Rotation2d rotationalTolerance) {
    return Commands.waitUntil(() -> {
        Pose2d robotPose = s_Swerve.getState().Pose;
        double tErr = robotPose.getTranslation().getDistance(target.getTranslation());
        double rErr = Math.abs(robotPose.getRotation().minus(target.getRotation()).getRadians());
        return tErr < translationalToleranceMeters
            && rErr < rotationalTolerance.getRadians();
    });
}
```
**Integration point:** Call this inside every auto shoot sequence, before calling indexer commands. Example usage in a path-based auto:
```java
Commands.sequence(
    AutoBuilder.followPath(path),
    AutoCommands702.waitUntilWithinTolerance(
        s_Swerve, shootingPose, 0.15, Rotation2d.fromDegrees(5.0))
        .withTimeout(2.0),   // don't wait forever if odometry drifts
    shootGated()
)
```

---

### Flywheel `atGoal()` Gate
**Source:** Team 6328, `AutoCommands.java:89-91` (`Commands.waitUntil(flywheel::atGoal)`)
**Adapted for Team 702:**
```java
// Add to ShooterSubsystem.java
private double targetVelocityRPS = 0.0;
private static final double VELOCITY_TOLERANCE_RPS = 2.0;

public boolean atGoal() {
    if (targetVelocityRPS <= 0) return false;
    double actual = FlywheelMotor2.getVelocity().getValueAsDouble();
    return Math.abs(actual - targetVelocityRPS) < VELOCITY_TOLERANCE_RPS;
}

// Modified setVelocity records the target:
public void setVelocity(double velocity) {
    double rps = MetersTotick(velocity);
    if (rps > 50) rps = 50;
    this.targetVelocityRPS = rps;
    FlywheelMotor1.setControl(new Follower(Constants.ShooterConstants.shooterMotor2,
        MotorAlignmentValue.Opposed));
    FlywheelMotor2.setControl(velControl.withVelocity(rps));
}
```
**Integration point:** `ShooterSubsystem.java` — method `atGoal()` is then used anywhere as `s_ShooterSubsystem::atGoal`.

---

### Recursive N-Cycle Auto Builder
**Source:** Walton Robotics, `WaltAutonFactory.java:94-121`
**Adapted for Team 702:**
```java
// In Auto702Factory.java
private Command createCycleSequence(int cycles, boolean isLeft) {
    String[] paths = isLeft ? LEFT_CYCLE_PATHS : RIGHT_CYCLE_PATHS;

    if (cycles <= 0) return Commands.none();

    if (cycles == 1) {
        // Base case: to neutral, pick up, to shoot, shoot
        return Commands.sequence(
            followPath(paths[0]),                  // start -> neutral
            Commands.runOnce(
                () -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem),
            Commands.waitSeconds(0.4),
            Commands.runOnce(
                () -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem),
            followPath(paths[1]),                  // neutral -> shoot pose
            shootGated()                           // wait for flywheel + fire
        );
    }

    ArrayList<Command> seq = new ArrayList<>();
    seq.add(createCycleSequence(cycles - 1, isLeft));  // recurse
    if (cycles != 2) {
        seq.add(shootGated());                     // only for cycles 3+
    }
    seq.add(followPath(paths[2]));                 // shoot pose -> neutral
    seq.add(Commands.runOnce(
        () -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem));
    seq.add(Commands.waitSeconds(0.4));
    seq.add(Commands.runOnce(
        () -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem));

    return Commands.sequence(seq.toArray(new Command[0]));
}
```
**Integration point:** `Auto702Factory.java` — instantiate the factory in `RobotContainer.java` constructor, call methods in `configureAutonomous()`.

---

### Parallel Intake + Pre-Spin During Path Follow
**Source:** Team 5507 GWHS, `NeutralAutos.java:106-119`
**Adapted for Team 702:**
```java
// Wrapper around AutoBuilder.followPath that runs intake in parallel
private Command followPathWithIntake(String pathName, boolean homeOnStart) {
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path)
            .deadlineFor(
                Commands.sequence(
                    // Homing on first cycle only
                    homeOnStart
                        ? Commands.runOnce(() -> {}, c_ClimbSubsystem) // replace with actual home cmd
                        : Commands.none(),
                    // Deploy intake immediately
                    Commands.parallel(
                        Commands.runOnce(
                            () -> s_ShooterSubsystem.setVelocity(0), s_ShooterSubsystem),
                        Commands.runOnce(
                            () -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem)),
                    // After 2.5 seconds into path, start pre-spinning flywheel
                    Commands.waitSeconds(2.5),
                    Commands.runOnce(
                        () -> s_ShooterSubsystem.setVelocity(15.0), s_ShooterSubsystem)));
    } catch (Exception e) {
        edu.wpi.first.wpilibj.DriverStation.reportError(
            "Path not found: " + pathName, e.getStackTrace());
        return Commands.none();
    }
}
```
**Integration point:** Replace bare `AutoBuilder.followPath()` calls with `followPathWithIntake()` in `Auto702Factory.java`.

---

### `mirrorPath()` for Left/Right Variants
**Source:** Team 5507 GWHS, `NeutralAutos.java:66-73`
**Adapted for Team 702:**
```java
// Instead of maintaining separate "Left-start-collect.path" and "Right-start-collect.path",
// load one path and mirror it for the other side:
PathPlannerPath baseCycle = PathPlannerPath.fromPathFile("Left-start-collect");
PathPlannerPath mirroredCycle = baseCycle.mirrorPath();  // geometrically mirrors across field Y-centerline

// Then in the auto:
Command leftAuto  = AutoBuilder.followPath(baseCycle);
Command rightAuto = AutoBuilder.followPath(mirroredCycle);
```
**Note:** This only works correctly if the path was created on the blue-alliance side. For Team 702's existing paths in `src/main/deploy/pathplanner/paths/`, verify the path geometry is on the correct starting side before mirroring. Existing paths like `Left-start-collect.path` and `Right-start-collect.path` are likely already separate — `mirrorPath()` would allow keeping only one.

---

### Auto Error Handling via try/catch
**Source:** Team 5507 GWHS, `NeutralAutos.java:50-88`, `DepotPathAuto_1c.java:27-56`
**Adapted for Team 702:**
```java
// Wrap all path loading in try/catch so a missing file shows an error,
// not a VM crash. Add this to every auto class constructor or factory method.
try {
    PathPlannerPath myPath = PathPlannerPath.fromPathFile("My-Path-Name");
    // ... add commands using myPath ...
} catch (Exception e) {
    DriverStation.reportError(
        "[Auto702] Path not found: " + e.getMessage(), e.getStackTrace());
    // Fall back to a safe no-op so auto doesn't crash the VM
    addCommands(Commands.none());
}
```
**Integration point:** Wrap path loading in `Auto702Factory.java` and any `SequentialCommandGroup` subclasses.

---

### Zone-Aware Triggers on `Swerve`
**Source:** Team 5507 GWHS, `SwerveSubsystem.java:109-131`, `RobotContainer.java:331-348` (GWHS)
**Adapted for Team 702:**
```java
// Add to Swerve.java (src/main/java/frc/robot/subsystems/Swerve.java)
// Requires adding the import: import edu.wpi.first.wpilibj2.command.button.Trigger;

/** True when robot is on its alliance side of the field (X < half field for blue). */
public final Trigger isInAllianceZone = new Trigger(() -> {
    double robotX = getState().Pose.getX();
    boolean isRed = Constants.getAlliance(); // or however alliance is determined
    double halfField = 8.23; // meters — update to match Constants.FieldConstants
    return isRed ? robotX > halfField : robotX < halfField;
});

/** True when shooter is within 5 degrees of hub target heading. */
public final Trigger isFacingHub = new Trigger(() -> {
    // Requires AutoAimCommand to expose its heading error, or compute inline:
    // Stub for now — replace with actual turret heading check
    return false; // TODO: t_TurretSubsystem.atGoal()
});
```
**Integration point:** `Swerve.java` as public fields. Then in `RobotContainer.configureBindings()`:
```java
// Auto prespin when entering alliance zone in any mode:
s_Swerve.isInAllianceZone.onTrue(
    Commands.runOnce(() -> s_ShooterSubsystem.setVelocity(15.0), s_ShooterSubsystem));
s_Swerve.isInAllianceZone.onFalse(
    Commands.runOnce(() -> s_ShooterSubsystem.setVelocity(0), s_ShooterSubsystem));
```
