// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.wpilib.math.util.MathUtil;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.driverstation.Joystick;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
// import frc.robot.LimelightHelpersCameronEdition;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;



public class TurretSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.TurretConstants.TurretMotorID, Constants.CAN_BUS);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  // private LimelightSubsystem limelightSubsystem;
  /** Creates a new ClimbSubsystem. */
   public TurretSubsystem() {
    // limelightSubsystem = limesub; 
    //m_encoderFR.setSimDevice(SimDevice.create("encoder"));
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
      turretConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            //turretConfig.
            

            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getDegreesToticks(Constants.TurretConstants.forwardLimit);
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getDegreesToticks(Constants.TurretConstants.reverseLimit);

            var s_slot0Configs = turretConfig.Slot0;
              s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
              s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
              s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
              s_slot0Configs.kP = 7.5; // An error of 1 rps results in 0.11 V output
              s_slot0Configs.kI = 0; // no output for integrated error
              s_slot0Configs.kD = 0.1; // no output for error derivative
              
    
            var motionMagicConfigs = turretConfig.MotionMagic;
              motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps 40
              motionMagicConfigs.MotionMagicAcceleration = 100; // 80 Target acceleration of 160 rps/s (0.5 seconds)
              motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

              var motorOutput = turretConfig.MotorOutput;
              motorOutput.Inverted = InvertedValue.Clockwise_Positive;

      Motor.getConfigurator().apply(turretConfig);
    
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    double angle = getAngleAsDouble();
    Telemetry.log("turretangle", angle);
    RobotContainer.CurrentAngle = angle;
    
    // Telemetry.log("limelightrobotyaw", getLimelightYaw());
    Telemetry.log("turretAngleTicks", getAngleAsTicks());
    Constants.TurretConstants.angle = getAngleAsDouble();

   
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.setThrottle(-speed);
    Telemetry.log("hiiiiiii", true);
    
    
  }

  public Command run(DoubleSupplier input){
    
    return this.runEnd(() -> this.setSpeed(input.getAsDouble()), () -> this.setSpeed(0.0));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(Math.toRadians(getAngleAsDouble()));
    
    
  }

  public double getAngleAsDouble() {
    return getTicksToDegrees(Motor.getPosition().getValueAsDouble());
  }

  
  public double getAngleAsTicks() {
    return Motor.getPosition().getValueAsDouble();
  }

  public double getTicksToDegrees(double ticks){
    return ticks * 360/Constants.TurretConstants.TurretConversionRate;
  }

 public double getDegreesToticks(double degrees){
    return degrees * Constants.TurretConstants.TurretConversionRate/360;
  }

  public void setAngle(double degrees){
    Motor.setPosition(getDegreesToticks(degrees));
  }

  public double getVel(){
    return Motor.getVelocity().getValueAsDouble();
  }
  public void setVoltage(double Voltage){
    Motor.setVoltage(Voltage);
  }

  
  // public double getLimelightYaw(){
  //   double limelightMeasurement = limeli;
    
  //   return limelightMeasurement;
  // }

  public void goToAngle(double angle){
    if(angle > Constants.TurretConstants.forwardLimit)angle = Constants.TurretConstants.forwardLimit;
    if(angle < Constants.TurretConstants.reverseLimit)angle = Constants.TurretConstants.reverseLimit;
    Motor.setControl(motionMagic.withPosition(getDegreesToticks(angle)));
    RobotContainer.TurretGoal = angle;
  }

  public void goToAngleOffset(double angleOffset){
    double angle = angleOffset + getAngleAsDouble();
    if(angle > Constants.TurretConstants.forwardLimit)angle = Constants.TurretConstants.forwardLimit;
    if(angle < Constants.TurretConstants.reverseLimit)angle = Constants.TurretConstants.reverseLimit;
    
    Motor.setControl(motionMagic.withPosition(getDegreesToticks(angle)));
  
  }

}