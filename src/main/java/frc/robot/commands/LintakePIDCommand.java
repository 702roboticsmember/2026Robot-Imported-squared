// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LintakePIDCommand extends Command {
  LIntakeSubsystem lIntakeSubsystem;
  boolean wiggle;
  PIDController lPidController = new PIDController(Constants.LintakeConstants.kP, Constants.LintakeConstants.kI, Constants.LintakeConstants.kD);
  PIDController rPidController = new PIDController(Constants.LintakeConstants.kP, Constants.LintakeConstants.kI, Constants.LintakeConstants.kD);
  double setpoint;
  Timer timer = new Timer();
  /** Creates a new LintakePIDCommand. */
  public LintakePIDCommand(LIntakeSubsystem sub, boolean wiggle, double setpoint) {
    lIntakeSubsystem = sub;
    addRequirements(sub);
    this.wiggle = wiggle;
    this.setpoint = setpoint;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (wiggle) {
      timer.reset();
      timer.start();
    } else {
      lPidController.setSetpoint(setpoint);
      rPidController.setSetpoint(-setpoint);
    }
    lPidController.setTolerance(0.1);
    rPidController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wiggle) {
      lPidController.setSetpoint(setpoint + (Math.sin(timer.get() * Constants.LintakeConstants.wigglePeriod) * Constants.LintakeConstants.wiggleAmplitude));
      rPidController.setSetpoint(setpoint + (Math.sin(timer.get() * Constants.LintakeConstants.wigglePeriod) * Constants.LintakeConstants.wiggleAmplitude));
    }

    double lspeed = lPidController.calculate(lIntakeSubsystem.getLTicks());
    double rspeed = rPidController.calculate(lIntakeSubsystem.getRTicks());
    lIntakeSubsystem.setLSpeed(lspeed);
    lIntakeSubsystem.setRSpeed(rspeed);
    Telemetry.log("L Lintake Speed", lspeed);
    Telemetry.log("R Lintake Speed", rspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lIntakeSubsystem.setBoth(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lPidController.atSetpoint() && rPidController.atSetpoint();
  }
}
