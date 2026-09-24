
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  TalonFX HoodMotor = new TalonFX(Constants.HoodConstants.HoodMotor, Constants.CAN_BUS);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  /** Creates a new HoodSubsystem. */
 

  public HoodSubsystem() {
    
        TalonFXConfigurator talonFXConfigurator = HoodMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.HoodConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.HoodConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.HoodConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = degToTick(Constants.HoodConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.HoodConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = degToTick(Constants.HoodConstants.reverseLimit);

    var s_slot0Configs = config.Slot0;
    s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    s_slot0Configs.kP = 5; // An error of 1 rps results in 0.11 V output
    s_slot0Configs.kI = 0; // no output for integrated error
    s_slot0Configs.kD = 0.1; // no output for error derivative
    
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    var motorOutput = config.MotorOutput;
    motorOutput.Inverted =InvertedValue.Clockwise_Positive;

    talonFXConfigurator.apply(config);
    setDegrees(Constants.HoodConstants.initialAngle);

    
  }

  public void setDegrees(double degrees){
    HoodMotor.setPosition(degToTick(degrees));
  }

  public void setSpeed(double speed) {
    HoodMotor.setThrottle(speed);
  }
  public double tickToDeg(double tick){
    return tick *Constants.HoodConstants.conversion * 360;
  }
  public double degToTick(double deg){
    return deg * 1/(Constants.HoodConstants.conversion * 360);
  }
  public double getHoodAngle(){
    return tickToDeg(HoodMotor.getPosition().getValueAsDouble());
  }                  

  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setSpeed(speed.getAsDouble());
    }, () -> {
      setSpeed(0);
    }, this);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Telemetry.log("HoodAngle", getHoodAngle());
     
  } 

  public void goToAngle(double angle){
    Telemetry.log("HoodgoTo", tickToDeg(degToTick(angle)));
    if(angle > Constants.HoodConstants.forwardLimit)angle = Constants.HoodConstants.forwardLimit;
    if(angle < Constants.HoodConstants.reverseLimit)angle = Constants.HoodConstants.reverseLimit;
    HoodMotor.setControl(motionMagic.withPosition(degToTick(angle)));
    
  }
}
