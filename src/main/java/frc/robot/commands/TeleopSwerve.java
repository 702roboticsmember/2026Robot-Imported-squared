package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command2.Command;

public class TeleopSwerve extends Command {
    private Swerve swerveSubsystem;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private BooleanSupplier robotCentricSupplier;
    private BooleanSupplier BlueAlliance;
     Rotation2d offset;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier BlueAlliance) {
        this.swerveSubsystem = s_Swerve;
        addRequirements(s_Swerve);
        this.BlueAlliance = BlueAlliance;

        this.translationSupplier = translationSup;
        this.strafeSupplier = strafeSup;
        this.rotationSupplier = rotationSup;
        this.robotCentricSupplier = robotCentricSup;
        if(BlueAlliance.getAsBoolean())
                offset = new Rotation2d(Math.toRadians(0));
        else 
                offset = new Rotation2d(Math.toRadians(180));
    }

    @Override
    public void initialize(){
        if(BlueAlliance.getAsBoolean())
                offset = new Rotation2d(Math.toRadians(0));
        else 
                offset = new Rotation2d(Math.toRadians(180));
    
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSupplier.getAsDouble(),
                Constants.CONTROLLER_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.CONTROLLER_DEADBAND);
        

        /* Drive */
        swerveSubsystem.driveAdjustedHeading(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSupplier.getAsBoolean(),
                Constants.Swerve.isOpenLoop,
                offset);
    }
}