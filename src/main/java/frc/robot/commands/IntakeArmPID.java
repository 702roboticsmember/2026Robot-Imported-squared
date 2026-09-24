// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeArmSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeArmPID extends Command {
  /** Creates a new IntakeArmPID. */
  private IntakeArmSubsytem i_subsystem;
  private PIDController controller = new PIDController(Constants.IntakeConstants.kP,Constants.IntakeConstants.kI,Constants.IntakeConstants.kD);
  private double targetAngle;
  public IntakeArmPID(double targetAngle, IntakeArmSubsytem i_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    this.i_subsystem = i_subsystem;
    addRequirements(i_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(targetAngle);
    controller.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(i_subsystem.getArmAngle());
    i_subsystem.setArmSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i_subsystem.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
