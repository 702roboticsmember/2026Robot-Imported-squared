// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import frc.robot.commands.LintakePIDCommand;
import frc.robot.commands.PointToPointPID;
import frc.robot.commands.TurretRotateManualCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LIntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;

/** Add your docs here. */
public class Autos {
    public static Command simpleAuto(Swerve s_Swerve, LIntakeSubsystem lIntakeSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.setPose(new Pose2d(3.418, 4.041, Rotation2d.ZERO))),
            new LintakePIDCommand(lIntakeSubsystem, false, -7));
    }
    public static Command leftIntake(TurretSubsystem turretSubsystem, Swerve swerve, LIntakeSubsystem lIntakeSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
            new WaitCommand(5),
            // new TurretRotateManualCommand(() -> 0.5, turretSubsystem).withDeadline(new WaitCommand(5)),
            new PointToPointPID(swerve, new Pose2d(3.418, 7.406, Rotation2d.ZERO)),
            new LintakePIDCommand(lIntakeSubsystem, false, -7),
            new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(1)),
            new PointToPointPID(swerve, new Pose2d(7.445, 7.406, Rotation2d.CCW_PI_2))
        );
    }
}
