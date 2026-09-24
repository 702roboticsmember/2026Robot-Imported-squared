// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import frc.robot.Constants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointToPointPID extends Command {

  private PIDController X = new PIDController(Constants.PointToPointPIDConstants.tP, Constants.PointToPointPIDConstants.tI, Constants.PointToPointPIDConstants.tD);
  private PIDController Y = new PIDController(Constants.PointToPointPIDConstants.tP, Constants.PointToPointPIDConstants.tI, Constants.PointToPointPIDConstants.tD);
  private PIDController A = new PIDController(Constants.PointToPointPIDConstants.aP, Constants.PointToPointPIDConstants.aI, Constants.PointToPointPIDConstants.aD);
  private Pose2d setpoint;
  private Swerve Swerve;
  /** Creates a new PointToPointPID. */
  public PointToPointPID(Swerve Swerve, Pose2d setpoint) {
    addRequirements(Swerve);
    this.Swerve = Swerve;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    X.setSetpoint(setpoint.getX());
    X.setTolerance(Constants.PointToPointPIDConstants.tTolerance);
    Y.setSetpoint(setpoint.getY());
    Y.setTolerance(Constants.PointToPointPIDConstants.tTolerance);
    A.setSetpoint(setpoint.getRotation().getDegrees());
    A.setTolerance(Constants.PointToPointPIDConstants.aTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = X.calculate(Swerve.getPose().getX());
    double yVal = Y.calculate(Swerve.getPose().getY());
    double aVal = A.calculate(Swerve.getPose().getRotation().getDegrees());

    Swerve.drive(new Translation2d(xVal, yVal), aVal, true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return X.atSetpoint() && Y.atSetpoint() && A.atSetpoint();
  }
}
