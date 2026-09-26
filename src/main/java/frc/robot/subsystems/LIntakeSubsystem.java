// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants;

public class LIntakeSubsystem extends SubsystemBase {
  TalonFX Lmotor = new TalonFX(Constants.LintakeConstants.lmotorID, Constants.CAN_BUS);
  TalonFX Rmotor = new TalonFX(Constants.LintakeConstants.rmotorID, Constants.CAN_BUS);
  /** Creates a new LIntakeSubsystem. */
  public LIntakeSubsystem() {
    Rmotor.setControl(new Follower(Constants.LintakeConstants.lmotorID, MotorAlignmentValue.Opposed));
    Lmotor.configNeutralMode(NeutralModeValue.Brake);
    Rmotor.configNeutralMode(NeutralModeValue.Brake);
  }

  public void setSpeed(double speed) {
    Lmotor.setThrottle(speed);
  }

  public double getTicks() {
    return Lmotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    Telemetry.log("Lintake Pos", getTicks());
    // This method will be called once per scheduler run
  }

  public Command LIntkakeTest(DoubleSupplier axis) {
    return Commands.runEnd(
      () -> {
        setSpeed(axis.getAsDouble()*0.5);
      },
      () -> {
        setSpeed(0);
      },
      this);
  }
}
