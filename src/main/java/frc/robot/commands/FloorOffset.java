// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.FloorIndexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorOffset extends Command {
  private FloorIndexerSubsystem f_FloorIndexerSubsystem;
  private double xi = 0;
  private double setPosition = 0;
  /** Creates a new FloorOffset. */
  public FloorOffset(FloorIndexerSubsystem f_FloorIndexerSubsystem, double setPosition) {
    this.f_FloorIndexerSubsystem = f_FloorIndexerSubsystem;
    this.setPosition = setPosition;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xi = f_FloorIndexerSubsystem.getIndexerPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    f_FloorIndexerSubsystem.goTo(xi + setPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
