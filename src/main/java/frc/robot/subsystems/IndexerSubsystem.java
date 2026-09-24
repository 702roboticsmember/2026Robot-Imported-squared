// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Robot;

public class IndexerSubsystem extends SubsystemBase {

  TalonFX indexMotorPrimary = new TalonFX(Constants.IndexerConstants.indexMotorTurret1, Constants.CAN_BUS);
  // TalonFX indexMotorSecondary = new TalonFX(Constants.IndexerConstants.indexMotorTurret2, Constants.CAN_BUS);

  private MotionMagicVelocityVoltage velControl = new MotionMagicVelocityVoltage(0);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  public double speed;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    System.out.println("Indexer Sub Init");
    //Primary
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerConstants.ENABLE_CURRENT_LIMIT;
    
    // indexMotorSecondary.setControl(new Follower(Constants.IndexerConstants.indexMotorTurret1, MotorAlignmentValue.Aligned));

    var slot0Configs2 = config.Slot0;
    slot0Configs2.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs2.kP = 0.11; // A position error of 2.5 rotations results in 12 V output
    slot0Configs2.kI = 0; // no output for integrated error
    slot0Configs2.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    // //Secondary
    //         TalonFXConfigurator talonFXConfiguratorSecondary = indexMotorSecondary.getConfigurator();
    // TalonFXConfiguration configSecondary = new TalonFXConfiguration();
    
    // var CurrentLimitsSecondary = config.CurrentLimits;
    // CurrentLimitsSecondary.StatorCurrentLimit = Constants.IndexerConstants.STATOR_CURRENT_LIMIT_SECONDARY;
    // CurrentLimitsSecondary.SupplyCurrentLimit = Constants.IndexerConstants.CURRENT_LIMIT_SECONDARY;
    // CurrentLimitsSecondary.StatorCurrentLimitEnable = Constants.IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT_SECONDARY;
    // CurrentLimitsSecondary.SupplyCurrentLimitEnable = Constants.IndexerConstants.ENABLE_CURRENT_LIMIT_SECONDARY;
    // //apply
    // talonFXConfigurator.apply(config);
    // talonFXConfiguratorSecondary.apply(configSecondary);
    indexMotorPrimary.getConfigurator().apply(config);
    // indexMotorSecondary.getConfigurator().apply(config);
  }

  // public void setSpeedSecondary(double speed){
  //   indexMotorSecondary.set(speed);
  // }

  public void setSpeedPrimary(double speed){
    indexMotorPrimary.setThrottle(speed);
    // indexMotorSecondary.setControl(new Follower(Constants.IndexerConstants.indexMotorTurret1, MotorAlignmentValue.Aligned));
  }
  public double getPrimaryDeg(){
  return primaryTickToDeg(indexMotorPrimary.getPosition().getValueAsDouble());
  }
  // public double getSecondaryDeg(){
  //   return secondaryTickToDeg(indexMotorSecondary.getPosition().getValueAsDouble());
  // }
 
//   public double secondaryTickToDeg(double Tick){
//     return Tick * 1/1;
// }
 public double primaryTickToDeg( double tick){
  return tick * 1/1;
 }
  @Override
  public void periodic() {
    Telemetry.log("indexvel", indexMotorPrimary.getVelocity().getValueAsDouble());
    //  Telemetry.log("indexSvel", indexMotorSecondary.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public Command spin(DoubleSupplier speedPrimary) {
    return Commands.runEnd(() -> {
      setSpeedPrimary(speedPrimary.getAsDouble()); //Constants.IndexerConstants.PrimarySpeed
      // setSpeedSecondary(speedSecondary.getAsDouble()); //Constants.IndexerConstants.SecondarySpeed
    }, () -> {
      setSpeedPrimary(0);
      // setSpeedSecondary(0);
    }, this);
  }

  public void setVelocity(double vel){
    indexMotorPrimary.setControl(velControl.withVelocity(vel));
    // indexMotorSecondary.setControl(new Follower(Constants.IndexerConstants.indexMotorTurret1, MotorAlignmentValue.Aligned));
  }

  public double getVelocity(){
    return indexMotorPrimary.getVelocity().getValueAsDouble();
  }
  public void setpose(double pos){
    indexMotorPrimary.setControl(motionMagic.withPosition(pos));
    // indexMotorSecondary.setControl(new Follower(Constants.IndexerConstants.indexMotorTurret1, MotorAlignmentValue.Aligned));
  }
    //FlywheelMotor2.setControl(velControl.withVelocity(rps));
  
}
