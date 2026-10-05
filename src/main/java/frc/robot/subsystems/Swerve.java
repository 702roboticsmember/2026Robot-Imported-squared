package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import java.io.IOException;
import java.util.Optional;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.limelightvision.PoseEstimate;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.math.estimator.PoseEstimator;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.SubsystemBase;

/**
 * Our main drive subsystem
 */
public class Swerve extends SubsystemBase {
    
    public SwerveModule[] swerveModules;
    public static Pigeon2 gyro;
    // public  RobotConfig config;
     public TurretSubsystem t_TurretSubsystem;
    public LimelightSubsystem limelightSubsystem;
    
    public static SwerveDrivePoseEstimator swervePoseEstimator;
    


    public Swerve(TurretSubsystem t_TurretSubsystem, LimelightSubsystem limelightSubsystem) {
        this.t_TurretSubsystem = t_TurretSubsystem;
        this.limelightSubsystem = limelightSubsystem;
      
        //limelightMeasurement =  LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightBack);
        gyro = new Pigeon2(Constants.Swerve.GyroId, Constants.CAN_BUS);
        gyro.reset();
        // try {
        //     config = RobotConfig.fromGUISettings();
        // } catch (IOException | ParseException e) {
        //     e.printStackTrace();
        // }

        swerveModules = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    
        
        if(gyro.isConnected()){
            swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d());
        }else{
            swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, new Rotation2d(Math.toRadians(0)), getModulePositions(), new Pose2d());
        }
        
        // AutoBuilder.configure(
        //     this::getPose, // Robot pose supplier
        //     this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getRobotRelativeSpeeds, // ChassisVelocities supplier. MUST BE ROBOT RELATIVE
        //     (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisVelocities. Also optionally outputs individual module feedforwards
        //     Constants.Swerve.PATHPLANNER_FOLLOWER_CONFIG,
        //     config,
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                        
        //             Optional<Alliance> alliance = MatchState.getAlliance();
        //             if ((alliance).isPresent()) {
        //                 if((alliance).get() == Alliance.RED){
        //                     RobotContainer.BLUE_ALLIANCE = false;
        //                     }else{
        //                         RobotContainer.BLUE_ALLIANCE = true;
        //                     }
        //               return (alliance).get() == Alliance.RED;
        //             }
        //             return false;
        //           },
        //           this // Reference to this subsystem to set requirements
        //   );
    }

    private void driveRobotRelative(ChassisVelocities speeds) {
        var SwerveModuleVelocitys = Constants.Swerve.KINEMATICS.toSwerveModuleVelocities(speeds);
        SwerveModuleVelocitys = SwerveDriveKinematics.desaturateWheelVelocities(
                SwerveModuleVelocitys, Constants.Swerve.MAX_SPEED);

        for (int i = 0; i < SwerveModuleVelocitys.length; i++) {
            swerveModules[i].setDesiredState(SwerveModuleVelocitys[i], Constants.Swerve.isOpenLoop);
        }
    }

    private ChassisVelocities getRobotRelativeSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisVelocities(getModuleStates());
    }

    private void resetPose(Pose2d startingPosition) {
        Telemetry.log("xi", startingPosition.getX());
        Telemetry.log("yi", startingPosition.getY());
        Telemetry.log("ai", startingPosition.getRotation().getDegrees());
        swervePoseEstimator.resetPosition(
                new Rotation2d(Math.toRadians(gyro.getYaw().getValueAsDouble())),
                this.getModulePositions(),
                startingPosition);
                
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        translation = translation.rotateBy(getHeading().unaryMinus());
        ChassisVelocities chassisVelocities = new ChassisVelocities(
                        translation.getX(),
                        translation.getY(),
                        rotation);
        
        // if(true) {
        //     chassisVelocities = chassisVelocities.toFieldRelative(getHeading());
        // }
        SwerveModuleVelocity[] SwerveModuleVelocitys = 
                Constants.Swerve.KINEMATICS.toSwerveModuleVelocities(chassisVelocities);
                
        SwerveModuleVelocitys = SwerveDriveKinematics.desaturateWheelVelocities(SwerveModuleVelocitys, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(SwerveModuleVelocitys[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveAdjustedHeading(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Rotation2d TurretOffset) {
        translation = translation.rotateBy(getHeading().plus(TurretOffset).unaryMinus());
        ChassisVelocities chassisVelocities = new ChassisVelocities(
                        translation.getX(),
                        translation.getY(),
                        rotation);
        
        // if(true) {
        //     chassisVelocities = chassisVelocities.toFieldRelative(getHeading().plus(TurretOffset));
        // }
        SwerveModuleVelocity[] SwerveModuleVelocitys = 
                Constants.Swerve.KINEMATICS.toSwerveModuleVelocities(chassisVelocities);
                
        SwerveModuleVelocitys = SwerveDriveKinematics.desaturateWheelVelocities(SwerveModuleVelocitys, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(SwerveModuleVelocitys[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleVelocity[] desiredStates, boolean isOpenLoop) {
        var SwerveModuleVelocitys = SwerveDriveKinematics.desaturateWheelVelocities(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(SwerveModuleVelocitys[mod.moduleNumber], isOpenLoop);
        }
    }

    public SwerveModuleVelocity[] getModuleStates() {
        SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        // return getPose().getRotation();
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void setHeading(Rotation2d heading) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }
    public void setPosition(Pose2d pose) {
        zeroHeading();
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void zeroHeading() {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
       
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble())
                : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    // public double getAcc() {
    //     return gyro.getAccelFullScaleRangeG();
    // }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }


    // public double getGyroVelX(){
    //     return gyro.getAccel;
    // }
    // public double getGyroVelY(){
    //     return gyro.getRobotCentricVelocityY();
    // }
    // public double getGyroVelZ(){
    //     return gyro.getRobotCentricVelocityZ();
    // }

    public Pose2d limelightTurretPoseAdjustedToRobot(Pose2d pose){
        //Pose2d pose2 = null;
        double y = Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  -Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = pose.getRotation().minus(new Rotation2d(Math.toRadians(Constants.TurretConstants.angle)));
        
       Pose2d returnpose = new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ), a);
       Telemetry.log("ogPosex", pose.getX());
           Telemetry.log("ogPosey", pose.getY());
           Telemetry.log("ogheading", pose.getRotation().getDegrees());
       Telemetry.log("adjPosex", returnpose.getX());
           Telemetry.log("adjPosey", returnpose.getY());
           Telemetry.log("adjheading", a.getDegrees());
           return returnpose;
        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }

   public  Pose2d RobotPoseAdjustedTolimelightTurret(Pose2d pose){
        double y = -Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = pose.getRotation();
        Rotation2d t = this.t_TurretSubsystem.getAngle();
        Pose2d returnpose = new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ),  a.plus(t));
        Telemetry.log("2adjPosex", returnpose.getX());
           Telemetry.log("2adjPosey", returnpose.getY());
           Telemetry.log("2adjheading", a.getDegrees());
       return returnpose;

        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }
   

    // public void setHeadingToField(){
    //     Rotation2d rotate = LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret).pose.getRotation();
    //     if (this.limelightMeasurement != null){
    //     setHeading(rotate);
    //     }
    // }
    // public void setposetoField(){
        
    //     if (this.limelightMeasurement != null){
    //     setPose(limelightMeasurement.pose);
    //     }
    // }

public void addmt1VisionMeasurement(PoseEstimate mt1){
        boolean doRejectUpdate = false;
        if (mt1 != null){
         if(mt1.reportedTagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].getDistanceToCamera() > 1.5)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.reportedTagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(mt1.avgTagDistanceMeters > 5.8)
      {
        doRejectUpdate = true;
      }
      if(Double.isNaN(mt1.reportedStdDevs[0]) || Double.isNaN(mt1.reportedStdDevs[1]) || Double.isNaN(mt1.reportedStdDevs[2])){
            mt1.reportedStdDevs[0] = .05;
            mt1.reportedStdDevs[1] =.05;
            mt1.reportedStdDevs[2] = .1;
        }

      if(!doRejectUpdate)
      {
        
        swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(mt1.reportedStdDevs[0] * 5,mt1.reportedStdDevs[1] * 5, mt1.reportedStdDevs[2]/5));
        swervePoseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
            Telemetry.log("ran", true);
             Telemetry.log("dist",  mt1.avgTagDistanceMeters);
      }
        }
    }

    public void addmt2VisionMeasurement(PoseEstimate mt2){
      boolean doRejectUpdate = false; 
 
   
  // if our angular velocity is greater than 360 degrees per second, ignore vision updates
//   if(Math.abs(gyro.) > 360)
//   {
//     doRejectUpdate = true;
//   }
  if(mt2.reportedTagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(mt2.reportedStdDevs[0], mt2.reportedStdDevs[1], mt2.reportedStdDevs[2]));
    swervePoseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
  }
    
    }



    @Override
    public void periodic() {
        //  LimelightHelpersCameronEdition.SetRobotOrientation(Constants.limelightConstants.limelightTurret, this.RobotPoseAdjustedTolimelightTurret(swervePoseEstimator.getEstimatedPosition()).getRotation().getDegrees(), 0, 0, 0, 0, 0);
        limelightSubsystem.SetRobotOrientation(this.RobotPoseAdjustedTolimelightTurret(swervePoseEstimator.getEstimatedPosition()).getRotation().getDegrees());
        if(Double.isNaN(swervePoseEstimator.getEstimatedPosition().getX())){
            swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d());
        }
        var limelightMeasurement =  limelightSubsystem.getBackPose();
        var limelightMeasurementTurret =  limelightSubsystem.getPoseEstimateMt1();
        //if(gyro.isConnected())swervePoseEstimator.updateWithTime(Timer.getTimestamp(), getGyroYaw(), getModulePositions());
        swervePoseEstimator.updateWithTime(Timer.getMonotonicTimestamp(), getGyroYaw(), getModulePositions());
       
        Telemetry.log("Pose", swervePoseEstimator.getEstimatedPosition());
        


        Telemetry.log("gyro", getHeading().getDegrees() );
        
        
        //Telemetry.log("Acc",this.getAcc());
        for (SwerveModule mod : swerveModules) {
            Telemetry.log("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            Telemetry.log("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            Telemetry.log("Mod " + mod.moduleNumber + " Velocity", mod.getState().velocity);
        }

        if (limelightMeasurementTurret != null){
            if(limelightMeasurementTurret.pose != null && limelightMeasurementTurret.pose.getRotation() != null){
              Pose2d pose = limelightTurretPoseAdjustedToRobot(limelightMeasurementTurret.pose);
              
              limelightMeasurementTurret.pose = pose;
             addmt1VisionMeasurement(limelightMeasurementTurret); 
              
            }
           }
           ChassisVelocities speed = getRobotRelativeSpeeds();
           Constants.Swerve.speeds = speed;
        Telemetry.log("chassisx", speed.vx);
        Telemetry.log("chassisy", speed.vy);
        if (limelightMeasurement != null){   
            addmt1VisionMeasurement(limelightMeasurement); 
        }
    // Constants.Swerve.Robotpose = swervePoseEstimator.getEstimatedPosition();
    // Constants.TurretConstants.turretPose2d = RobotPoseAdjustedTolimelightTurret(swervePoseEstimator.getEstimatedPosition());
    
 
} 
    
   
}
    

