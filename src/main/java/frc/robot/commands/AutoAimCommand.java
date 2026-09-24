// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;

import com.limelightvision.Limelight;

import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimCommand extends Command {
 
  private TurretSubsystem t_TurretSubsystem ;
  private HoodSubsystem h_HoodSubsystem;
  private ShooterSubsystem s_ShooterSubsystem;
  private LimelightSubsystem limelightSubsystem;
  

  private double g = Constants.PhysicsConstants.gravity;
  private double h = Constants.PhysicsConstants.HubHeight;
  private double i = Constants.PhysicsConstants.BallIntialHeight;

  private Translation2d poi;

  private boolean isFixed;
  private boolean passing;
  private boolean Alliance;
  private boolean Hub;
  private BooleanSupplier BlueAlliance;
 
  boolean angleRight;

  

  
  /** Creates a new AutoAimCommand. */
  public AutoAimCommand(Translation2d poi, TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem, BooleanSupplier BlueAlliance, LimelightSubsystem limelightSubsystem) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    this.poi = poi;
    this.isFixed = true;
    this.Hub = false;
    this.BlueAlliance = BlueAlliance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AutoAimCommand(boolean Hub, TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem, BooleanSupplier BlueAlliance) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
     this.BlueAlliance = BlueAlliance;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    this.isFixed = true;
    this.poi = BlueAlliance.getAsBoolean() ? Constants.Locations.BLUEHUB.location: Constants.Locations.REDHUB.location;
    this.Hub = Hub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AutoAimCommand(TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem, boolean passing, BooleanSupplier BlueAlliance) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    
    this.BlueAlliance = BlueAlliance;
    this.isFixed = false;
    this.passing = passing;
    if(passing){
    this.poi = BlueAlliance.getAsBoolean() ? Constants.Locations.BLUELEFT.location: Constants.Locations.REDLEFT.location;
    this.h = 0;
    }
    else this.poi = RobotContainer.currentPOI.location;
    this.Hub = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance = BlueAlliance.getAsBoolean();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isFixed){
    this.poi = RobotContainer.currentPOI.location;
    }

    
    
    Pose2d Robotpose = Swerve.swervePoseEstimator.getEstimatedPosition();
    Pose2d pose = RobotPoseAdjustedTolimelightTurret(Robotpose);
    //limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret);

    //checkAngle();
    //SmartDashboard.putBoolean("good to shoot", angleRight);
    //Constants.Swerve.good = ()-> Good();

    if(passing){
      if(Alliance){
        if(pose.getY() > Constants.Locations.CENTER.location.getY())
        this.poi = Constants.Locations.BLUELEFT.location;
        else
        this.poi = Constants.Locations.BLUERIGHT.location;
      }else{
        if(pose.getY() > Constants.Locations.CENTER.location.getY())
        this.poi = Constants.Locations.REDRIGHT.location;
        else
        this.poi = Constants.Locations.REDLEFT.location;

      }
    }
    if(Hub){
    if(Alliance){
      this.poi = Constants.Locations.BLUEHUB.location;
    }else{
      this.poi = Constants.Locations.REDHUB.location;
    }
    }
    
    // Stationary shoot code
    // double RobotBasedAngle = getTurretAngleToHub(pose).getDegrees();
    // double distance = getDistance(pose);
    // double vy = CalculateVy(distance);
    // double vx = CalculateVx(distance, vy);
    // double vs = CalculateVs(vx, vy, 0, 0);
    // double shootAngle = CalculateShootAngle(vx, vy, 0, 0);

    // Shoot on the move code
    double Dx = getDistance(pose);
    double vy = CalculateVy(Dx);
   
    double vrz = getVrz(pose);
    double vrx = getVrx(pose);
    double t = timeTillTarget(vy);
    double angleOffset = CalculateOffset(vrz, Dx, t);
    double distance = CalculateShotDistance(angleOffset, Dx);
    double vx = CalculateVx(distance, vy);
    double vs = CalculateVs(vx, vy, vrx, vrz, angleOffset);
    double shootAngle = CalculateShootAngle(vx, vy, vrx, vrz, angleOffset);
    double RobotBasedAngle = getTurretAngleToHub(pose).getDegrees();
    RobotBasedAngle += angleOffset;

    if(RobotBasedAngle > Constants.TurretConstants.forwardLimit){
      RobotBasedAngle -=360;
    }
    if(RobotBasedAngle < Constants.TurretConstants.reverseLimit){
      RobotBasedAngle +=360;
    }
    Telemetry.log("shootAngle", shootAngle );
    Telemetry.log("heading", Robotpose.getRotation().getDegrees());
    // Telemetry.log("posex", Robotpose.getX());
    // Telemetry.log("posey", Robotpose.getY());
    // Telemetry.log("odtes", RobotBasedAngle);
    // Telemetry.log("shootSpeed", vs);
    // Telemetry.log("shootSpeedx", vx);
    Telemetry.log("shootSpeedy", vy);
    Telemetry.log("vrx", vrx);
    Telemetry.log("vrz", vrz);
     Telemetry.log("t", t);
    
    
    // Telemetry.log("poseturretangle2", pose.getRotation().getDegrees());
    Telemetry.log("poseturretdist", distance);
    // Telemetry.log("poitx", poi.getX());
    // Telemetry.log("poity", poi.getY());
    
    t_TurretSubsystem.goToAngle(RobotBasedAngle);
    double offby = t_TurretSubsystem.getAngleAsDouble() - RobotBasedAngle;
    if(Math.abs(offby) > 3){
      Constants.Swerve.good = ()-> false;
    }else{
      Constants.Swerve.good = ()-> true;
    }
    
      //Linear regression.
      //double output = vs * 2.00429 + 1.91073;//TODO Calibrate based on input velocity vs ball actual velocity
      double output = 0.450191 * Math.pow(vs, 3) -9.12405 * Math.pow(vs, 2) + 62.95509* vs -131.96832;
      //double output = 0.811385 * Math.pow(vs, 3) -16.69722 * Math.pow(vs, 2) + 116.04568* vs -256.58627;//y=0.811385x^{3}-16.69722x^{2}+116.04568x-256.58627
      //double output = 1.74188 * vs + 3.91243;
      //double output = 3.64532 * Math.pow(vs, 0.76359);
      //2.2492 + 0.56157
      //Quartic regression


      //double output = 0.00918838 * Math.pow(vs, 4) - 0.199587 * Math.pow(vs, 3) + 1.45776*Math.pow(vs, 2) - 2.20965* vs+ 5.3498;
      Telemetry.log("output", output);
      Telemetry.log("vs", vs);
      s_ShooterSubsystem.setVelocity(output);
      h_HoodSubsystem.goToAngle(shootAngle);
    
    
    //s_ShooterSubsystem.setVelocity(vs * 2.1492 + 0.56157);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ShooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
    public boolean isFinished(){
    return false;
  }

 

  private double getDistance(Pose2d Pose) {
    return Pose.getTranslation().getDistance(poi);
  }

  private Rotation2d getAngleToHub(Pose2d Pose){
    return poi.minus(Pose.getTranslation()).getAngle().get();
  }

  public Rotation2d getTurretAngleToHub(Pose2d pose){
    return getAngleToHub(pose).minus(pose.getRotation());
  }


  /**
   * getVrx:
   *  unfinished 
   * @return nothing right now.
   */
  public double getVrx(Pose2d pose){
    
    Rotation2d angle = new Rotation2d(Math.toRadians(360)).minus(getTurretAngleToHub(pose));
    ChassisVelocities speed = Constants.Swerve.speeds;
    double x = speed.vx;
    double y = speed.vy;
    Translation2d vel = new Translation2d(
      x,
      y);
      

    return vel.rotateBy(angle).getX();
  }
  
  /**
   * getVrz:
   *  unfinished 
   * @return nothing right now.
   */

  public double getVrz(Pose2d pose){
    Rotation2d angle = new Rotation2d(Math.toRadians(360)).minus(getTurretAngleToHub(pose));
    ChassisVelocities speed = Constants.Swerve.speeds;
    double x = speed.vx;
    double y = speed.vy;
    Translation2d vel = new Translation2d(
      x,
      y);
  
    return -vel.rotateBy(angle).getY();
  }
  /**
   * RobotPoseAdjustedTolimelightTurret
   * @param pose the current position of the robot relative to the field(Blue-side relative)
   * @return Position of the center of the turret relative to the field.
   */
  public static Pose2d RobotPoseAdjustedTolimelightTurret(Pose2d pose){
        double y = -Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = pose.getRotation();
        Pose2d returnpose = new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ),  a);
        Telemetry.log("2adjPosex", returnpose.getX());
           Telemetry.log("2adjPosey", returnpose.getY());
           Telemetry.log("2adjheading", a.getDegrees());
       return returnpose;

        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }

  /**CalculateVy
   * @param Vrz Robot velocity in z direction (direction perpendicular to hub)
   * @param Dx (meters) value that is the distance from hub and not desired distance of shot.
   * @param t time it takes for the ball to reach target
   * @return angle offset of the turret in degrees.
   */
  public double CalculateOffset(double Vrz, double Dx, double t){
    double Input = (Vrz* t)/Dx;

    if (Double.isNaN(Math.atan(Input))){
      return 0;
      //TODO Constant based on data
    }

    return Math.toDegrees(Math.atan(Input));
  }

  /**
   * Calculates the new distance the robot has to shoot based on time it takes for the ball to reach target
   * @param Vrz Robot velocity in z direction (direction perpendicular to hub)
   * @param Dx (meters) value that is the distance from hub and not desired distance of shot.
   * @param t time it takes for the ball to reach target
   * @return the distance the ball must travel.
   */
  public double CalculateShotDistance(double Vrz, double Dx, double t){
    double angle = CalculateOffset(Vrz, Dx, t);
    return Dx/Math.cos(Math.toRadians(angle)) ;
  }

/**
   * Calculates the new distance the robot has to shoot based on time it takes for the ball to reach target
   * @param OffsetAngle
   * @param Dx
   * @return the distance the ball must travel.
   */
  public double CalculateShotDistance(double OffsetAngle, double Dx){
    return Dx/Math.cos(Math.toRadians(OffsetAngle));
  }

  /**CalculateVy
   *  @param Dx psudo value that takes in distance from hub and not desired distance of shot.
   * @return Vy (Vertical velocity) based on a preset formula to help reduce complexity.
   */
  public double CalculateVy(double Dx){
    return Math.clamp(0.098438*Dx + 5.81997, 5.8, 1000);//Originally a constant but I found adjusting the Vy and in turn the max height I can increase accuracy from afar.
  }

  public double getBasicVy(){
    return 6.2;
  }

  public double timeTillTarget(double Vy){
    double a = -g * 0.5;
    double b = Vy;
    double c = -(h-i);
    double Input = (b * b) - (4* a * c);
    if(Input < 0){
      Input = 0;//TODO Constant based on data
    }
    return ((-b) - Math.sqrt(Input))/ (2*a);
  }


  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h - i);
    double Input = (b * b) - 4*(a * c);
    if(Input < 0){
      Input = 0;//TODO Constant based on data
    }

    return (2*(a)) / ((-b) - Math.sqrt(Input));
  }

  public double CalculateVs(double Vx, double Vy, double Vrx, double Vrz, double angleOffset){
    double vrx = Vrx* Math.cos(Math.toRadians(angleOffset));
    double vrz = Vrz * Math.sin(Math.toRadians(angleOffset));
    double Input = ((Vx - vrx + vrz) * (Vx - vrx + vrz)) + (Vy * Vy);
    if (Input < 0){
      Input = 0;//TODO Constant based on data
    }
    return Math.sqrt(Input);
  }

  public double CalculateShootAngle(double Vx, double Vy, double Vrx, double Vrz, double angleOffset){
    double vrx = Vrx* Math.cos(Math.toRadians(angleOffset));
    double vrz = Vrz * Math.sin(Math.toRadians(angleOffset));
    double HoodAngle = Vy/(Vx - vrx + vrz);

   
    if(Vy/(Vx - vrx + vrz) == (Math.PI/2)){
      return 0;//TODO Constant based on data
    }
   
    else if( Vy/(Vx - vrx + vrz) == (-Math.PI/2) ){
      return 0;//TODO Constant based on data
    }
    else{
      return Math.toDegrees(Math.atan(HoodAngle));
    }
  }

  public Rotation2d getTurretAngle(Pose2d pose){
    return pose.getRotation();
  }

  public double getIMUYaw(){
    // return LimelightHelpersCameronEdition.getIMUData("limelight").gyroY;
    return limelightSubsystem.getIMUYaw();
  }
}
