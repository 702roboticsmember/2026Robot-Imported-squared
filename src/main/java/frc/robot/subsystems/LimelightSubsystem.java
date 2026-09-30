// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;

import com.limelightvision.Limelight;

import com.limelightvision.FiducialTarget;
import com.limelightvision.IMUData;
import com.limelightvision.IMUMode;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.MatchState;

import frc.robot.Constants;


public class LimelightSubsystem extends SubsystemBase {
  Limelight limelight = new Limelight(Constants.limelightConstants.limelightTurret);
  Limelight secondary = new Limelight(Constants.limelightConstants.limelightBack);
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    limelight.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
  }

  public LimelightResults getResults() {
    return limelight.getLatestResults();
  }

  public FiducialTarget[] getTargets() {
    return getResults().fiducialTargets;
  }
  public FiducialTarget getTarget() {
    return getTargets()[0];
  }
  public Pose2d getTargetPoseCameraSpace() {
    var targets = getTarget();
    return Limelight.toPose2D(targets.targetPoseCameraSpace);
  }
  public Pose2d getTargetPoseRobotSpace() {
    var targets = getTarget();
    return Limelight.toPose2D(targets.targetPoseRobotSpace);
  }
  public Pose2d getCameraPoseTargetSpace() {
    var targets = getTarget();
    return Limelight.toPose2D(targets.cameraPoseTargetSpace);
  }
  public IMUData getIMUData() {
    return getResults().imu;
  }

  public boolean isTargetAvailable() {
    return limelight.getTargetCount() > 0;
  }
  public double getTXDegrees() {
    return getTarget().txDegrees;
  }

  public double getTA() {
    return limelight.getTargetAreaPercent();
  }

  public double getIMUYaw() {
    return getIMUData().yawDegrees;
  }

  public PoseEstimate getPoseEstimateMt1() {
    Alliance alliance = MatchState.getAlliance().get();
    PoseEstimateType poseEstimateType = (alliance == Alliance.RED) ? PoseEstimateType.MT1_WPIRED : PoseEstimateType.MT1_WPIBLUE;
    return limelight.getPoseEstimate(poseEstimateType);
  }

  public PoseEstimate getBackPose() {
    Alliance alliance = MatchState.getAlliance().get();
    PoseEstimateType poseEstimateType = (alliance == Alliance.RED) ? PoseEstimateType.MT1_WPIRED : PoseEstimateType.MT1_WPIBLUE;
    return secondary.getPoseEstimate(poseEstimateType);
  }

  public PoseEstimate getPoseEstimateMt2() {
    Alliance alliance = MatchState.getAlliance().get();
    PoseEstimateType poseEstimateType = (alliance == Alliance.RED) ? PoseEstimateType.MT2_WPIRED : PoseEstimateType.MT2_WPIBLUE;
    return limelight.getPoseEstimate(poseEstimateType);
  }


  public void SetRobotOrientation(double YawDegrees) {
    limelight.setRobotOrientation(YawDegrees, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
