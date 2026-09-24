// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryEntry;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.GenericEntry;

import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(Constants.ClimbConstants.climbMotor, Constants.CAN_BUS);
  //private Servo ratchetServo = new Servo(0);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    System.out.println("Climb Sub Init");
    TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.ClimbConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.ClimbConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.ClimbConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.ClimbConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.ClimbConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = DistToTick(Constants.ClimbConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.ClimbConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = DistToTick(Constants.ClimbConstants.reverseLimit);

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

    var motorOutput = config.MotorOutput;
    motorOutput.Inverted =InvertedValue.Clockwise_Positive;
    motorOutput.NeutralMode = Constants.ClimbConstants.mode;

    talonFXConfigurator.apply(config);
  }

   public double getClimbAngle(){
    return TickToDist(motor.getPosition().getValueAsDouble());
  }

  public double TickToDist(double ticks){
    return ticks *1/1;
  }

  public double DistToTick(double dist){
    return dist * 1/1;
  }

  public void setSpeed(double speed){
    motor.setThrottle(speed);
  }
//   public void setServo(double angle){
//     ratchetServo.setAngle(angle);
//  }

  @Override
  public void periodic() {
    // Telemetry.log("ClimbAngle", getClimbAngle());
    Telemetry.log("ClimbAngle", getClimbAngle());
    // This method will be called once per scheduler run
  }

  public void goToPos(double pos){
    Telemetry.log("ClimbgoTo", TickToDist(DistToTick(pos)));
    if(pos > Constants.ClimbConstants.forwardLimit)pos = Constants.ClimbConstants.forwardLimit;
    if(pos < Constants.ClimbConstants.reverseLimit)pos = Constants.ClimbConstants.reverseLimit;
    motor.setControl(motionMagic.withPosition(DistToTick(pos)));
    
  }

  public void goToPosOffset(double posOff){
    double pos = getClimbAngle() + posOff;
    if(pos > Constants.ClimbConstants.forwardLimit)pos = Constants.ClimbConstants.forwardLimit;
    if(pos < Constants.ClimbConstants.reverseLimit)pos = Constants.ClimbConstants.reverseLimit;
    motor.setControl(motionMagic.withPosition(DistToTick(pos)));
    
  }

}
