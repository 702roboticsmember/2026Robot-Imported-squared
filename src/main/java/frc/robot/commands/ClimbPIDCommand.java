// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPIDCommand extends Command {
  /** Creates a new ClimbPIDCommand. */
  private ClimbSubsystem c_subsystem;
  private PIDController controller = new PIDController(Constants.ClimbConstants.kP, Constants.ClimbConstants.kI, Constants.ClimbConstants.kD);
  private double TargetAngle; 
  
  public ClimbPIDCommand(double TargetAngle, ClimbSubsystem c_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.TargetAngle = TargetAngle;
    this.c_subsystem = c_subsystem;
    addRequirements(c_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(TargetAngle);
    controller.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(c_subsystem.getClimbAngle());
    c_subsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
