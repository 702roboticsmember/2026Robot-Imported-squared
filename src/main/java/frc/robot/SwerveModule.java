package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,
            Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    // private final SlewRateLimiter openLoopLimiter = new SlewRateLimiter(.05);
    // private final SlewRateLimiter closedLoopLimiter = new SlewRateLimiter(.05);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.CAN_BUS);
        angleEncoder.getConfigurator().apply(Robot.CTRE_CONFIGS.swerveCANcoderConfig);

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.CAN_BUS);
        angleMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CAN_BUS);
        driveMotor.getConfigurator().apply(Robot.CTRE_CONFIGS.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleVelocity desiredState, boolean isOpenLoop) {
        desiredState = desiredState.optimize(getState().angle);
        this.setAngleSpeed(desiredState.angle.getRotations());
        this.setSpeed(desiredState, isOpenLoop);
    }

    /**
     * @param rotations the number of rotations of a {@link Rotation2d}
     */
    private void setAngleSpeed(double rotations) {
        angleMotor.setControl(anglePosition.withPosition(rotations));
    }

    private void setSpeed(SwerveModuleVelocity desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double toOutput = desiredState.velocity / Constants.Swerve.MAX_SPEED;

            // toOutput = openLoopLimiter.calculate(toOutput);

            driveDutyCycle.Output = toOutput;

            driveMotor.setControl(driveDutyCycle);
        } else {
            double toOutputVelocity = Conversions.MPSToRPS(desiredState.velocity,
                    Constants.Swerve.WHEEL_CIRCUMFERENCE);

            //toOutputVelocity = closedLoopLimiter.calculate(toOutputVelocity);

            driveVelocity.Velocity = toOutputVelocity;
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.velocity);

            driveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleVelocity getState() {
        return new SwerveModuleVelocity(
                Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(),
                        Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble()));
    }
}