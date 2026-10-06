// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
  public static TalonFX motor = new TalonFX(Constants.IndexerConstants.ROLLER_MOTOR_ID, Constants.CAN_BUS);
  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {}
  public void setSpeed(double speed) {
    motor.setThrottle(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
