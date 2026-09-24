// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.wpilib.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.wpilib.math.util.MathUtil;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX FlywheelMotor1 = new TalonFX(Constants.ShooterConstants.shooterMotor1, Constants.CAN_BUS);
  private TalonFX FlywheelMotor2 = new TalonFX(Constants.ShooterConstants.shooterMotor2, Constants.CAN_BUS);

  private MotionMagicVelocityVoltage velControl = new MotionMagicVelocityVoltage(0);
  //private TalonFX TurretMotor = new TalonFX(Constants.ShooterConstants.angleMotor);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    System.out.println("Shooter Sub Init");
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.ShooterConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = Constants.ShooterConstants.kV ; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = Constants.ShooterConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = Constants.ShooterConstants.kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = Constants.ShooterConstants.kI; // no output for integrated error
    slot0Configs.kD = Constants.ShooterConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    
    var limits = talonFXConfigs.CurrentLimits;

    limits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimit = Constants.ShooterConstants.CURRENT_LIMIT;
    limits.StatorCurrentLimitEnable = Constants.ShooterConstants.ENABLE_STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimitEnable = Constants.ShooterConstants.ENABLE_CURRENT_LIMIT;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    var motorOutput = talonFXConfigs.MotorOutput;
    motorOutput.Inverted =InvertedValue.Clockwise_Positive;

    var talonFXConfigs2 = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs2 = talonFXConfigs2.Slot0;
    slot0Configs2.kS = Constants.ShooterConstants.kS * 2; // Add 0.25 V output to overcome static friction
    slot0Configs2.kV = Constants.ShooterConstants.kV * 1; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs2.kA = Constants.ShooterConstants.kA * 2; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs2.kP = Constants.ShooterConstants.kP * 4; // A position error of 2.5 rotations results in 12 V output
    slot0Configs2.kI = Constants.ShooterConstants.kI * 1; // no output for integrated error
    slot0Configs2.kD = Constants.ShooterConstants.kD * 1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    
    var limits2 = talonFXConfigs2.CurrentLimits;

    limits2.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
    limits2.SupplyCurrentLimit = Constants.ShooterConstants.CURRENT_LIMIT;
    limits2.StatorCurrentLimitEnable = Constants.ShooterConstants.ENABLE_STATOR_CURRENT_LIMIT;
    limits2.SupplyCurrentLimitEnable = Constants.ShooterConstants.ENABLE_CURRENT_LIMIT;
   

    var motionMagicConfigs2 = talonFXConfigs2.MotionMagic;
    motionMagicConfigs2.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs2.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    var motorOutput2 = talonFXConfigs2.MotorOutput;
    motorOutput2.Inverted =InvertedValue.Clockwise_Positive;


    FlywheelMotor1.getConfigurator().apply(talonFXConfigs);
    FlywheelMotor2.getConfigurator().apply(talonFXConfigs2);
  FlywheelMotor1.setControl(new Follower(Constants.ShooterConstants.shooterMotor2, MotorAlignmentValue.Opposed));
    
  }

  public void setSpeed(double speed) {
    FlywheelMotor2.setVoltage(speed * 12);
    FlywheelMotor1.setVoltage(-speed * 12);
  }
   //public double tickToDeg("")

//   public void setVelocity(double velocity){
//     FlywheelMotor1.setControl(velControl.withVelocity(velocity));
//   }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Telemetry.log("ShooterVel", FlywheelMotor1.getVelocity().getValueAsDouble());
    Telemetry.log("ShooterVel2", FlywheelMotor2.getVelocity().getValueAsDouble());
  }

  public double tickToRotation(double tick){
    return tick *Constants.ShooterConstants.conversion;
  }
  public double rotationsToTick(double rotations){
    return rotations * 1.0/(Constants.ShooterConstants.conversion);
  }
  public double tickToMeters(double tick){
    return Conversions.RPSToMPS(tickToRotation(tick), 0.319176);
  }
  public double MetersTotick(double meters){
    return rotationsToTick(Conversions.MPSToRPS(meters, 0.319176));
  }


  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setSpeed(speed.getAsDouble());
    }, () -> {
      setSpeed(0);
    }, this);
  }
  /**
   * 
   * @param velocity in meters per second.
   */
  public void setVelocity(double velocity){
    double rps = MetersTotick(velocity);
    if(rps>50){
        rps = 50;
    }
    Telemetry.log("veltorps", rps);
    //SmartDashboard.putBoolean("test", );
    FlywheelMotor1.setControl(new Follower(Constants.ShooterConstants.shooterMotor2, MotorAlignmentValue.Opposed));
    FlywheelMotor2.setControl(velControl.withVelocity(rps));
  }
}
