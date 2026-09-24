// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.LimelightSubsystemRight;
import frc.robot.subsystems.Swerve;

public class AlignCommand extends Command {
  boolean interrupted;

  

  private PIDController RotatePID = new PIDController(
      Constants.AutoAimConstants.kP,
      Constants.AutoAimConstants.kI,
      Constants.AutoAimConstants.kD);
 
  
  
  Swerve s_Swerve;
  LimelightSubsystem l_LimelightSubsystem;
  Rotation2d headingprev;
  final double x;
  final double z;
  final double ry;

  

  /** Creates a new AutoAim. */
  public AlignCommand(double x, double z, double ry, LimelightSubsystem l_LimelightSubsystem, Swerve s_Swerve) {
    
    this.l_LimelightSubsystem = l_LimelightSubsystem;
    this.s_Swerve = s_Swerve;






    this.x = x;
    this.z = z;
    this.ry = ry;

    addRequirements(s_Swerve);
    addRequirements(l_LimelightSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.robotCentric = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Tx =  l_LimelightSubsystem.getCameraPoseTargetSpace().getX();
    double Tz =  l_LimelightSubsystem.getCameraPoseTargetSpace().getY();
    double distance = new Translation2d(Tx, Tz).getDistance(new Translation2d(x, z));
    // boolean inzone = Math.abs(Tx) < 0.3;
    
    RotatePID.setSetpoint(ry);
    RotatePID.setTolerance(1);

    

    Telemetry.log("distance", distance);
    

    
    boolean Target =  l_LimelightSubsystem.isTargetAvailable();
    // double value = TranslatePID.calculate(Tx);
    // double result = Math.copySign(Math.abs(value) + 0.01, value); 
    // double Tranlate = (Target && !TranslatePID.atSetpoint()  ? Math.clamp(value, -0.47, 0.47) : 0);
    // Telemetry.log("TPID", value);
    Telemetry.log("TTX", x);

    
    // double value1 = StrafePID.calculate(Tz);
    //double result1 = Math.copySign(Math.abs(value1) + 0.0955, value1); 
    // double Strafe = (Target && !StrafePID.atSetpoint()? Math.clamp(value1, -0.47, 0.47) : 0);
    // Telemetry.log("SPID", value1);
    Telemetry.log("STZ", z);
    //Cameron Trux Team 702 :3
    //double angle = Math.tanh(x/z);

    double a =  l_LimelightSubsystem.getTXDegrees();
    // double tx = l_LimelightSubsystem.getTargetX();
    double value2 =  RotatePID.calculate(a);
    //double result2 = Math.copySign(Math.abs(value2) + 0.0955, value2); 
    double Rotate = (Target && !RotatePID.atSetpoint() ? Math.clamp(value2, -0.17, 0.17) : 0);
    
    // if(Rotate > 0){
    //   if(tx.getAsDouble() > 12){
    //     Rotate = 0;
    //   }
    // }
    // if(Rotate < 0){
    //   if(tx.getAsDouble() < -17){
    //     Rotate = 0;
    //   }
    // }
    Telemetry.log("RRY", a);
    Telemetry.log("RPID", Rotate);
    // s_Swerve.drive(
    //             new Translation2d(Strafe, -Tranlate).times(Constants.Swerve.MAX_SPEED),
    //             Rotate * Constants.Swerve.MAX_ANGULAR_VELOCITY,
    //             !true,
    //             true);
    //             Telemetry.log("RRPID", Rotate* (1 + Strafe));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.robotCentric = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RotatePID.atSetpoint();
  }
}
