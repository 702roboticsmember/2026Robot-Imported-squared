package frc.robot;





import java.util.function.BooleanSupplier;

import com.limelightvision.Limelight;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.util.PathPlannerLogging;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.XboxController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.units.measure.Time;
import org.wpilib.smartdashboard.Field2d;
//import org.wpilib.smartdashboard.SendableChooser;
//import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
// import org.wpilib.command2.button.CommandXboxController;
// import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.JoystickButton;
import org.wpilib.command2.button.POVButton;
import org.wpilib.command2.button.Trigger;
import frc.robot.Constants.Locations;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.FloorOffset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurretRotateManualCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FloorIndexerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LIntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   
    //Subsystems
    private final IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem c_ClimbSubsystem = new ClimbSubsystem();
    private final IndexerSubsystem i_IndexerSubsystem = new IndexerSubsystem();
    private final IntakeArmSubsytem i_IntakeArmSubsystem = new IntakeArmSubsytem();
    private final FloorIndexerSubsystem f_FloorIndexerSubsystem = new FloorIndexerSubsystem();
    private final TurretSubsystem t_TurretSubsystem = new TurretSubsystem();
    private final HoodSubsystem h_HoodSubsystem = new HoodSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
   // private final LEDSubsystem l_LEDSubsystem = new LEDSubsystem();
    private final XboxController driver = new XboxController(0);
    private final LIntakeSubsystem lIntakeSubsystem = new LIntakeSubsystem();




    //Driver Buttons
    private final JoystickButton armin = new JoystickButton(driver, Gamepad.Button.FACE_RIGHT.value);
    private final JoystickButton armPartial = new JoystickButton(driver, Gamepad.Button.FACE_DOWN.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, Gamepad.Button.FACE_LEFT.value);
    private final JoystickButton intakeIn = new JoystickButton(driver, Gamepad.Button.LEFT_BUMPER.value);
    private final JoystickButton resetGyro = new JoystickButton(driver, Gamepad.Button.FACE_UP.value);
    private final JoystickButton shoot = new JoystickButton(driver, Gamepad.Button.RIGHT_BUMPER.value);
    private final JoystickButton autoAimHUB = new JoystickButton(driver, Gamepad.Button.START.value);
    private final JoystickButton autoAimPASS = new JoystickButton(driver, Gamepad.Button.BACK.value);
    
    private final POVButton UP = new POVButton(driver, POVDirection.UP);
    private final POVButton DOWN = new POVButton(driver, POVDirection.DOWN);
    private final POVButton LEFT = new POVButton(driver, POVDirection.LEFT);
    private final POVButton RIGHT = new POVButton(driver, POVDirection.RIGHT);
    
    
    
    

    
   
    //Variables
    public static boolean BLUE_ALLIANCE = false;
    public static double power = 1;
    public static double max = 1;
    public static double TurretGoal = 0;
    public static double CurrentAngle = 0;
    public BooleanSupplier hoodUp = ()-> true;
    public static boolean robotCentric = false;
    // private final SendableChooser<Command> autoChooser;
    private final Selectable<Command> teamChooser;

    public static Locations currentPOI = Locations.BLUEHUB;
    private Field2d locField2d = new Field2d();
    public static DigitalInput ClimbSwitch = new DigitalInput(0);
    public static Trigger ClimbTrigger = new Trigger(()-> ClimbSwitch.get());
    //public static AnalogTrigger ClimbSwitch = new AnalogTrigger(0);
    

    public void debugLocations() {
        Telemetry.log("Currently Aiming at",
                String.format("%s ", currentPOI.label));
                locField2d.setRobotPose(new Pose2d(currentPOI.location, new Rotation2d(0)));
        Telemetry.log("current location", locField2d);
    }


    public static boolean getAlliance(){
        // TODO find method for getting alliance
        BLUE_ALLIANCE = true;
        return true;
    }


    private Command ArmOut(){
        return Commands.run(()->i_IntakeArmSubsystem.goToAngle(100), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3));
    }

    private Command ArmIn(){
        return new ParallelCommandGroup(Commands.run(()->i_IntakeArmSubsystem.goToAngle(5), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3)), 
        HoodDown(), 
        Commands.run(()-> t_TurretSubsystem.goToAngle(0), t_TurretSubsystem)).withDeadline(new WaitCommand(1));
    }

    private Command ArmPartial(){
        return Commands.run(()->i_IntakeArmSubsystem.goToAngle(60), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3));
    }
    
    
    private Command IntakeIn() {
        return new ParallelCommandGroup(
            
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem)
        ).withDeadline(new WaitCommand(0.3));
    }
    private Command IntakeOut() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(-0.3), f_FloorIndexerSubsystem),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(-0.5), i_IntakeArmSubsystem)
        );
    }
    private Command IntakeStop() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem),
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(0), f_FloorIndexerSubsystem)
        );
    }

    private Command IntakeOff() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem)
        );
    }

    private Command Shoot() {
        double jamtime;
        double reset;
        jamtime = Timer.getMonotonicTimestamp();
        reset = 0;
        return new SequentialCommandGroup(
            Commands.run(()->{
                if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        i_IndexerSubsystem.setVelocity(140);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
                    else {
                        i_IndexerSubsystem.setVelocity(0);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
             }, i_IndexerSubsystem, f_FloorIndexerSubsystem).withDeadline(new WaitCommand(0.1)),
             new WaitCommand(0.1),
             Commands.run(()->{
                 if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        i_IndexerSubsystem.setVelocity(140);
                        f_FloorIndexerSubsystem.setVelocity(60);
                    }
                    else {
                        i_IndexerSubsystem.setVelocity(0);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
             }, i_IndexerSubsystem, f_FloorIndexerSubsystem).withDeadline(new WaitCommand(0.1)),
             new WaitCommand(0.1),
            new ParallelCommandGroup(
                //TODO figure out what tf this does
            //Commands.run(()->i_IntakeSubsystem.setIntakeSpeed(0.5), i_IntakeSubsystem),
            // Commands.run(()->{
            //     if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
            //             i_IndexerSubsystem.setVelocity(140);
            //         }
            //         else {
            //             i_IndexerSubsystem.setVelocity(0);
            //         }
            //     if(i_IndexerSubsystem.getVelocity() < 5 || (f_FloorIndexerSubsystem.getVelocity() < 5 && f_FloorIndexerSubsystem.getVelocity() > 0) ){
                    
            //     }else{
            //         jamTime = Timer.getMonotonicTimestamp();
            //     }
            //         if(Timer.getMonotonicTimestamp() - jamTime < 1 && !l_lidarSubsystem.jam() && Timer.getMonotonicTimestamp() - reset > 0.5){
            //         if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
            //             f_FloorIndexerSubsystem.setVelocity(60);
            //         } else {
            //             f_FloorIndexerSubsystem.setFloorIndexSpeed(20);
            //         }
            //     }else{
            //         Telemetry.log("fixed", true);
            //         f_FloorIndexerSubsystem.setVelocity(-40);
        
            //     }
                

            // },
            // f_FloorIndexerSubsystem, i_IndexerSubsystem),
                new InstantCommand(() -> max = 0.13)
                ));
    }

    private Command ShootOff() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
            
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(-10), i_IndexerSubsystem),
            new FloorOffset(f_FloorIndexerSubsystem, -20),
            new InstantCommand(() -> power = 1),
            new InstantCommand(() -> max = 1)

           ).withDeadline(new WaitCommand(0.1)), new WaitCommand(0.1), new ParallelCommandGroup(
            
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(-0), i_IndexerSubsystem),
            new InstantCommand(() -> power = 1),
            new InstantCommand(() -> max = 1)

           ));
        
    }

    private Command ShootOG() {
        return new SequentialCommandGroup(
            Commands.run(()->i_IndexerSubsystem.setVelocity(100), i_IndexerSubsystem).withDeadline(new WaitCommand(0.25)),
        new ParallelCommandGroup(
           Commands.run(()->i_IndexerSubsystem.setVelocity(100), i_IndexerSubsystem),
           Commands.run(()->i_IntakeSubsystem.setIntakeSpeed(0.3), i_IntakeSubsystem),
           Commands.run(()->f_FloorIndexerSubsystem.setVelocity(80), f_FloorIndexerSubsystem)
            
           
           ));
        
    }

    // private Command ShootDeadline(){
    //     return Commands.waitUntil(()-> !l_lidarSubsystem.indexer_full());
    // }

    // private Command ShootDeadlineTime(){
    //     return new SequentialCommandGroup(new WaitCommand(1), ShootDeadline());
    // }
    

    private Command AutoAim() {
        Telemetry.log("autorun", true);
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, false, ()-> getAlliance());
    }

    private Command Nest() {
        return new InstantCommand(() -> h_HoodSubsystem.goToAngle(0));
    }
    private Command wrapLocationChange(Runnable r) {
        return Commands.runOnce(() -> {
            r.run();
            RobotContainer.this.debugLocations();
        });
    }

    private Command AimAtHub(){
        
            return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance());
        
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    private Command AimAtHubBlue(){
        
            return new AutoAimCommand(Constants.Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance(), limelightSubsystem);
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    private Command AimAtHubRed(){
            
            return new AutoAimCommand(Constants.Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance(), limelightSubsystem);
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    //  private Command AimAtHub(){
    //     if(Constants.getAlliance())return new AutoAimCommand(Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> true);
    //     else return new AutoAimCommand(Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()->true);
    // }

    private Command PASS(){
      
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, true, ()-> getAlliance());
    }

    private Command ExtendClimb(){
        return Commands.run(()->c_ClimbSubsystem.goToPos(Constants.ClimbConstants.extendedAngle), c_ClimbSubsystem);
    }

    private Command RetractClimb(){
        return Commands.run(()->c_ClimbSubsystem.goToPos(Constants.ClimbConstants.retractedAngle), c_ClimbSubsystem);
    }

    private Command HoodDown(){
        return new InstantCommand(()-> h_HoodSubsystem.goToAngle(Constants.HoodConstants.forwardLimit));
    }

    private Command AutoShoot() {
        return Shoot();
    }

    public Command ClimbAuto() {
        return new SequentialCommandGroup(ExtendClimb());
    }



    public Command ClimbDeadline(){
        return Commands.waitUntil(ClimbTrigger);
    }


    // public Command AutoIntake(){

    //     // return new ParallelCommandGroup(new AutoIntakeCommand(
    //     //     ()->LimelightHelpersCameronEdition.getTX(Constants.limelightConstants.limelightFront),
    //     //     ()->LimelightHelpersCameronEdition.getTA(Constants.limelightConstants.limelightFront), 
    //     //     ()-> LimelightHelpersCameronEdition.getTV(Constants.limelightConstants.limelightFront), 
    //     //     s_Swerve, 
    //     //     0.3, 
    //     //     i_IntakeSubsystem));
    //     return new ParallelCommandGroup(
    //         new AutoIntakeCommand(
    //             () -> limelightSubsystem.getTargetPoseCameraSpace().getX(),
    //             () -> limelightSubsystem.getTA(),
    //             () -> limelightSubsystem.isTargetAvailable(),
    //             s_Swerve,
    //             0.3,
    //             i_IntakeSubsystem));
    // }


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve(t_TurretSubsystem, limelightSubsystem);

    public RobotContainer() {
       System.out.println("Robot Container Instantiation Starting");
        
        
        Field2d field = new Field2d();
        Telemetry.log("Field", field);
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     field.setRobotPose(pose);
        // });
        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     field.getObject("target pose").setPose(pose);
        // });
        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     field.getObject("path").setPoses(poses);
        // });

        t_TurretSubsystem.setDefaultCommand(new TurretRotateManualCommand(() -> driver.getRightX(), t_TurretSubsystem));
        c_ClimbSubsystem.setDefaultCommand(new InstantCommand(()-> c_ClimbSubsystem.setSpeed(driver.getLeftTrigger() - driver.getRightTrigger()), c_ClimbSubsystem));
        // l_LEDSubsystem.setDefaultCommand(
        // new InstantCommand(()->
        // {
        //     l_LEDSubsystem.DoTheRainbow(false);
        //     l_LEDSubsystem.LEDScroll(8,0,true);
        //     l_LEDSubsystem.LEDScroll(8, 8, false);
            

        // }
        // , l_LEDSubsystem));
        
        // TODO swerve
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> Math.clamp(-driver.getLeftY() * power, -max, max) , 
        ()-> Math.clamp(-driver.getLeftX() * power, -max, max),
        ()-> Math.clamp(-driver.getRightX() * power, -max, max), 
        ()->false, ()-> getAlliance()));
        lIntakeSubsystem.setDefaultCommand(lIntakeSubsystem.LIntkakeTest(() -> driver.getRightY()));

        // s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> Math.clamp(-driver.getLeftY(), -max, max), null, null, UP, ClimbTrigger));

        // s_ShooterSubsystem.setDefaultCommand(new InstantCommand(()-> s_ShooterSubsystem.setSpeed(codriver.getLeftTriggerAxis()* 0.4) , s_ShooterSubsystem));

        // driver.setRumble(RumbleType.kBothRumble, 0.2);

       
    //     NamedCommands.registerCommand("Shoot", AutoShoot());
    //     NamedCommands.registerCommand("ShootOff", ShootOff());
    //     NamedCommands.registerCommand("AimAtHub", AimAtHub());
    //     NamedCommands.registerCommand("AimAtHubBlue", AimAtHubBlue());
    //     NamedCommands.registerCommand("AimAtHubRed", AimAtHubRed());

    // // NamedCommands.registerCommand("AimAndShoot", AimAndShoot());
    //     NamedCommands.registerCommand("IntakeNormal", IntakeIn());
    //     NamedCommands.registerCommand("StopIntake", IntakeStop());
    //     NamedCommands.registerCommand("IntakeOut", IntakeOut());
    //     // NamedCommands.registerCommand("AutoIntake", AutoIntake());
    //     NamedCommands.registerCommand("Climb", ClimbAuto());
    //     NamedCommands.registerCommand("ClimbDeadline", ClimbDeadline());
    //     NamedCommands.registerCommand("ClimbGrab", new InstantCommand());
    //     NamedCommands.registerCommand("ClimbRetract", RetractClimb());
    //     // NamedCommands.registerCommand("ShootDeadline", ShootDeadlineTime());
    //     //ClimbGrab
    //     NamedCommands.registerCommand("ArmOut", ArmOut());
    //     NamedCommands.registerCommand("ArmPartial", ArmPartial());
    //     NamedCommands.registerCommand("HoodDown", HoodDown());

      

        
        configureButtonBindings();
        //TODO ADD AUTO
        // Build an auto chooser. This will use Commands.none() as the default option.
        // autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new Selectable<>();
        
        // SmartDashboard.putData("Auto Chooser", autoChooser);
        Telemetry.log("Team Chooser", teamChooser);
        Telemetry.log("Input Distance", 0);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
       
        intakeOut.whileTrue(IntakeOut()); 
        intakeOut.onFalse(IntakeStop()); 
        intakeIn.whileTrue(IntakeIn());//new ParallelCommandGroup(new IntakeArmPID(0, i_IntakeArmSubsystem), new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0))));  
        intakeIn.onFalse(IntakeStop());
        autoAimHUB.whileTrue(AimAtHub());
        autoAimHUB.onFalse(HoodDown());
        autoAimPASS.whileTrue(PASS());
        autoAimPASS.onFalse(HoodDown());
        // armin.onTrue(ArmIn());
        // armPartial.whileTrue(ArmPartial());
        // armPartial.onFalse(ArmOut());
        
      
        shoot.whileTrue(Shoot());
  
        shoot.onFalse(ShootOff());

        UP.onTrue(wrapLocationChange(()-> nextAllianceLocation()));
        DOWN.onTrue(wrapLocationChange(()-> prevAllianceLocation()));
        RIGHT.onTrue(wrapLocationChange(()-> nextLocation()));
        LEFT.onTrue(wrapLocationChange(()-> prevLocation()));
        // armOut.onTrue(ArmOut());
        resetGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // COSTART.whileTrue((new TeleopSwerve(s_Swerve, 
        // ()-> -driver.getRawAxis(1) * power, 
        // ()-> -driver.getRawAxis(0) * power,
        // ()-> -driver.getRawAxis(4) * power, 
        // ()-> true)));
        
        //extendClimb.onTrue(ExtendClimb());
        //retractClimb.onTrue(RetractClimb());
        // CORIGHTBUMPER.whileTrue(Shoot());
        // //shoot.whileTrue(new InstantCommand(()-> hoodUp= ()-> false));
        // CORIGHTBUMPER.onFalse(ShootOff());


        // COSTART.whileTrue(AimAtHub());
        // COSTART.onFalse(HoodDown());
        // COBACK.whileTrue(PASS());
        // COBACK.onFalse(HoodDown());
        

        // cograbClimb.onTrue(GrabClimb());
        // coreleaseClimb.onTrue(ReleaseClimb());
        // COLEFTBUMPER.whileTrue(new ShootDistCommand(3.3, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));
        // CORIGHTBUMPER.whileTrue(new ShootDistCommand(8, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));

    }


    void nextLocation(){
        RobotContainer.currentPOI = RobotContainer.currentPOI.next();
    }

    void prevLocation(){
        RobotContainer.currentPOI = RobotContainer.currentPOI.prev();
    }

    void nextAllianceLocation(){
        RobotContainer.currentPOI = RobotContainer.currentPOI.AlianceNext(getAlliance());
    }

    void prevAllianceLocation(){
        RobotContainer.currentPOI = RobotContainer.currentPOI.AliancePrev(getAlliance());
    }


    
     public Command getAutonomousCommand() {

        // return new ParallelCommandGroup(
        //     //new InstantCommand(()->Swerve.gyro.reset()),
        //     autoChooser.getSelected()).andThen(new ParallelCommandGroup(ShootOff(), ArmOut(), IntakeOff()).withDeadline(new WaitCommand(1)));
           return Commands.none(); 
          }
    }
     




