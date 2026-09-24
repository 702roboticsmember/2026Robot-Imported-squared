// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoIntakeCommand extends Command {
  boolean interrupted;

  private PIDController AutoFollowPID = new PIDController(

      Constants.AutoFollowConstants.kP,
      Constants.AutoFollowConstants.kI,
      Constants.AutoFollowConstants.kD);

      private PIDController AutoAimPID = new PIDController(
        Constants.AutoAimConstants.kP,
        Constants.AutoAimConstants.kI,
        Constants.AutoAimConstants.kD);
  
  
  DoubleSupplier tx;
  DoubleSupplier ta;
  BooleanSupplier tv;
  Swerve s_Swerve;
  double turn;
  IntakeSubsystem i_IntakeSubsystem;

  /** Creates a new AutoIntakeCommand */
  public AutoIntakeCommand(DoubleSupplier tx, DoubleSupplier ta, BooleanSupplier tv, Swerve s_Swerve, double turn, IntakeSubsystem i_IntakeSubsystem) {
    this.ta = ta;
    this.tx = tx;
    this.tv = tv;
    this.s_Swerve = s_Swerve;
    this.i_IntakeSubsystem = i_IntakeSubsystem;
    this.turn = turn;
    
    addRequirements(s_Swerve, i_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = true;
    AutoFollowPID.setSetpoint(Constants.AutoFollowConstants.AutoFollowPIDSetpoint);
    AutoFollowPID.setTolerance(Constants.AutoFollowConstants.AutoFollowPIDTolerance);
    AutoAimPID.setSetpoint(Constants.AutoFollowConstants.AutoAimPIDSetpoint);
    AutoAimPID.setTolerance(Constants.AutoFollowConstants.AutoAimPIDTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(Constants.Swerve.swervePoseEstimator.getEstimatedPosition().)

    double a = ta.getAsDouble();
    boolean Target = tv.getAsBoolean();
    double value = AutoFollowPID.calculate(a);
    double result = value > 0 ? value + 0.0955 : value - 0.0955;
    double FollowPID = (Target ? Math.clamp(result, -0.87, 0.87) : 0);
    Telemetry.log("FPID", value);
    Telemetry.log("Fta", a);

    double x = tx.getAsDouble();

    double value2 = AutoAimPID.calculate(x);
    double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    // value > 0 ? value + 0.0955 : value - 0.0955;
    double AimPID = (Target ? Math.clamp(result2, -0.57, 0.57) : turn);
    // Telemetry.log("FollowPID", RobotContainer.FollowPID);
    Telemetry.log("Ftx", x);
    i_IntakeSubsystem.setIntakeSpeed(Math.clamp(FollowPID * 0.5 + 0.3, 0, 1));
    s_Swerve.drive(
                new Translation2d(FollowPID, 0).times(Constants.Swerve.MAX_SPEED),
                AimPID * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !true,
                false);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
  return
  AutoFollowPID.atSetpoint() &&
  AutoAimPID.atSetpoint();
   
  }
}
