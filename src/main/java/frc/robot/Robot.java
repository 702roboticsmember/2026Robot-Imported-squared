// Copyright (c) LAST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

// import org.wpilib.vision.stream.CameraServer;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.Field2d;

import com.limelightvision.Limelight;

// import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;

import frc.robot.Constants.TurretConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.Swerve;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private final Field2d turretField = new Field2d();
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // SmartDashboard.putData("turret field", turretField);

    // and put our
    // autonomous chooser on the dashboard.
    //CameraServer.startAutomaticCapture();
    // TODO limelight IMU
    // Limelight.SetIMUMode(Constants.limelightConstants.limelightTurret, 4);
    // Limelight.IMUMode = IMUMode.
    System.out.println("Robot Container Initializing");
    robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // turretField.setRobotPose(AutoAimCommand.RobotPoseAdjustedTolimelightTurret(Swerve.swervePoseEstimator.getEstimatedPosition()));
   
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();


    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
    
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    //RobotContainer.getAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     
  }


}
