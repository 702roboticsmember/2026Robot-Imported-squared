  // Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants;

public class FloorIndexerSubsystem extends SubsystemBase {
   private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  TalonFX FloorIndexMotor = new TalonFX(Constants.IndexerConstants.indexMotorFeeder, Constants.CAN_BUS);
  private MotionMagicVelocityVoltage velControl = new MotionMagicVelocityVoltage(0);

  /** Creates a new FloorIndexerSubsystem. */
  public FloorIndexerSubsystem() {
    
    TalonFXConfigurator talonFXConfigurator = FloorIndexMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.FLOOR_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.FLOOR_CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerConstants.FLOOR_ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerConstants.FLOOR_ENABLE_CURRENT_LIMIT;

    var s_slot0Configs = config.Slot0;
    s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    s_slot0Configs.kP = 4.8; // An error of 1 rps results in 0.11 V output
    s_slot0Configs.kI = 0; // no output for integrated error
    s_slot0Configs.kD = 0.1; // no output for error derivative

     var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    
    talonFXConfigurator.apply(config);

  }
  public void setFloorIndexSpeed(double speed) {
    FloorIndexMotor.setThrottle(speed);
  }
  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(
      () -> { setFloorIndexSpeed(speed.getAsDouble()); },
      () -> { setFloorIndexSpeed(0); },
      this
    );
  }

  public double tickToDeg(double ticks) {
    return ticks*Constants.IndexerConstants.gearingMultiplier;
  }
  public double degToTick(double deg) {
    return deg/Constants.IndexerConstants.gearingMultiplier;
  }
  public double getIndexerPos() {
    return FloorIndexMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity(){
    return FloorIndexMotor.getVelocity().getValueAsDouble();
  }

  public void goTo(double val){
    FloorIndexMotor.setControl(motionMagic.withPosition( val));
  }

  public void move(double val){
    FloorIndexMotor.setControl(motionMagic.withPosition(getIndexerPos() + val));
  }
  public void setVelocity(double val){
    FloorIndexMotor.setControl(velControl.withVelocity(val));
  }

  @Override
  public void periodic() {
    Telemetry.log("floorpos", getIndexerPos());
    Telemetry.log("floorvel", FloorIndexMotor.getVelocity().getValueAsDouble());


    // This method will be called once per scheduler run
  }
}
