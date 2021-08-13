/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import frc.robot.Constants.MechConstants;

import frc.robot.commands.auto.FailSafeAutoWithVelocity;
import frc.robot.commands.auto.FailSafeAutoBackward;
import frc.robot.commands.auto.AutoSequentialShooter;
import frc.robot.commands.auto.AutoForwardMotionMagic;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.LimelightCenter;
import frc.robot.commands.auto.LimelightShooter;
import frc.robot.commands.auto.SequentialShooter;
import frc.robot.commands.auto.SlalomPathArc;
import frc.robot.commands.auto.BouncePathArc;
import frc.robot.commands.auto.FINiteRechargeAutoPath;
import frc.robot.commands.auto.BarrelPathArc;
import frc.robot.oi.CONSTANTS_OI;
import frc.robot.oi.Logitech;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber  = new Climber();
  // private final ColorWheel colorWheel = new ColorWheel();
  private final BallFeed ballFeed  = new BallFeed();
  private final Limelight limelight = new Limelight();
  private final PixyCam pixy = new PixyCam();
  private final Hood hood = new Hood();

  // The robot's operator interface controllers are defined here...
  private final Joystick leftJoystick = new Joystick(1);
  private final Joystick rightJoystick = new Joystick(2);
  private final Logitech logitecController = new Logitech(0);

  //Define various autos to select from
  private final Command m_SlalomAuto = new SlalomPathArc(drivetrain);
  private final Command m_BarrelAuto = new BarrelPathArc(drivetrain);
  private final Command m_BounceAuto = new BouncePathArc(drivetrain);
  // private final Command m_autoCommand = new LimelightCenter(drivetrain);
  private final Command m_FailSafeBackward = new FailSafeAutoBackward(drivetrain, shooter, ballFeed);
  private final Command m_SequentialShooter = new AutoSequentialShooter(shooter, ballFeed, 4900);
  private final Command m_ShootThenBackup = new SequentialCommandGroup(
      new AutoSequentialShooter(shooter, ballFeed, 4900).withTimeout(10),
      new AutoForwardMotionMagic(drivetrain, -36));
  private final Command m_FINiteRechargeCommand = new FINiteRechargeAutoPath(limelight, drivetrain, shooter, ballFeed, hood);
  private final Command m_GalacticAuto = new GalacticSearch(drivetrain, intake, pixy, ballFeed);

  //Create a chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drivetrain, leftJoystick, rightJoystick);
  private final RunClimber runClimber = new RunClimber(climber, logitecController);
  //private final AnglePixy anglePixy = new AnglePixy(pixy);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Put some buttons on the dashboard
    configureDashboard();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithJoysticks);
    climber.setDefaultCommand(runClimber);
    //pixy.setDefaultCommand(anglePixy);
    // climber.setDefaultCommand(driveClimberDefault);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // autoCenter.whileHeld(m_autoCommand);
    
    //NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    
    // below done with Marc
    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    //camera.setResolution(144, 144);
     // public JoystickButton leftButton6 = new JoystickButton(leftJoystick, 6);
  
    // Buttons for Logitech controller defined here:
    final JoystickButton logitecLeftTrigger = new JoystickButton(logitecController, CONSTANTS_OI.XBOX_LEFT_TRIGGER);
    final JoystickButton logitecLeftBumper = new JoystickButton(logitecController, XboxController.Button.kBumperLeft.value);


    final JoystickButton logitecLeftStick = new JoystickButton(logitecController, XboxController.Button.kStickLeft.value);
    final JoystickButton logitecRightStick = new JoystickButton(logitecController, XboxController.Button.kStickRight.value);
    
    final JoystickButton logitecButtonX = new JoystickButton(logitecController, 1);
    final JoystickButton logitecButtonA = new JoystickButton(logitecController, 2);
    final JoystickButton logitecButtonB = new JoystickButton(logitecController, 3);
    final JoystickButton logitecButtonY = new JoystickButton(logitecController, 4);
    
    final JoystickButton logitecRightBumper = new JoystickButton(logitecController, XboxController.Button.kBumperRight.value);
    final JoystickButton logitecRightTrigger = new JoystickButton(logitecController, CONSTANTS_OI.XBOX_RIGHT_TRIGGER);
    
    final Trigger logitecDPadUp = new Trigger( () -> logitecController.getDPadState().equals(Logitech.DPadState.UP));
    final Trigger logitecDPadLeft = new Trigger( () -> logitecController.getDPadState().equals(Logitech.DPadState.LEFT));
    final Trigger logitecDPadRight = new Trigger( () -> logitecController.getDPadState().equals(Logitech.DPadState.RIGHT));
    final Trigger logitecDPadDown = new Trigger( () -> logitecController.getDPadState().equals(Logitech.DPadState.DOWN));

    final JoystickButton logitecBackButton = new JoystickButton(logitecController, CONSTANTS_OI.XBOX_BACK_BUTTON);
    final JoystickButton logitecStartButton = new JoystickButton(logitecController, CONSTANTS_OI.XBOX_START_BUTTON);

    // Joystick buttons defined here:
    final JoystickButton leftButton3 = new JoystickButton(leftJoystick, 3);
    final JoystickButton rightButton4 = new JoystickButton(rightJoystick, 4);

    final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);

    final JoystickButton rightButton5 = new JoystickButton(rightJoystick, 5);
    final JoystickButton rightButton6 = new JoystickButton(rightJoystick, 6);

    final JoystickButton rightButton7 = new JoystickButton(rightJoystick, 7);

    final JoystickButton rightButton13 = new JoystickButton(rightJoystick, 13);
    final JoystickButton rightButton12 = new JoystickButton(rightJoystick, 12);

    final JoystickButton leftButton2 = new JoystickButton(leftJoystick, 2);
    final JoystickButton rightButton2 = new JoystickButton(rightJoystick, 2);
    // public JoystickButton rightButton3 = new JoystickButton(rightJoystick, 3);
    // public JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);
 
    //Other non-OI Triggers defined here:
    final Trigger shooterRunning = new Trigger( () -> shooter.getShooterSpeed() > 100);
    final Trigger shooterStable = new Trigger( () -> shooter.isStable());



    // Linking buttons to commands here...
    //logitecButtonA.toggleWhenActive(new RunShooterPID(shooter, 4900));
    //logitecButtonX.toggleWhenActive(new RunShooterPID(shooter, 2400));
    //logitecButtonY.whileHeld(new RunBothWinches(climber, -1.0, -1.0));
    //logitecButtonB.whileHeld(new RunBothWinches(climber, 1.0, 1.0));

    //Run all feeds in reverse while held.  Stop all feeds when released
    //Allows operator to clear jams or release balls
    logitecButtonA.whenHeld(new RunIntakeAndBallFeedAndShooterFeed(intake, ballFeed, -.35,
        -MechConstants.kBFSpeed, -MechConstants.kSFSpeed),false);  //Speeds as previously determined

    //Run shooter wheel at selected speed. Continues until cancelled by B button
    //Changed these from toggle as with a toggle it is difficult for the operator to know what state it is in.
    Command ReadyForNearShot = new ParallelCommandGroup(new RunShooterPID(shooter,2400), new RetractHood(hood));
    Command ReadyForFarShot = new ParallelCommandGroup(new RunShooterPID(shooter,4000), new ExtendHood(hood));
    logitecLeftBumper.whenActive(ReadyForNearShot.withInterrupt(logitecButtonB::get));
    logitecRightBumper.whenActive(ReadyForFarShot.withInterrupt(logitecButtonB::get));

    //Launch power cells by running the ballfeed and shooterfeed wheels.  Stop when released.
    //Only fires if the button is held and the shooter is running.
    logitecRightTrigger.and(shooterRunning).whileActiveOnce(
          new RunBothFeeders(ballFeed, MechConstants.kBFSpeed, MechConstants.kSFSpeed),false);

    //Auto align to target using limelight then shoot.  Stop when released.
    //Moved to button Y, if we do like this better than manual we can move back to trigger. 
    logitecButtonY.and(shooterRunning).whileActiveOnce(
        new SequentialCommandGroup(new LimelightAutoAim(limelight, drivetrain),
        new RunBothFeeders(ballFeed, MechConstants.kBFSpeed, MechConstants.kSFSpeed)),false);

    //Extend intake and run intake and ball feed while held.  Stop feeds and raise intake when released.
    //Allows operator to pick up balls
    logitecLeftTrigger.whenHeld(new SequentialCommandGroup(new ExtendIntake(intake), 
        new RunIntakeAndBallFeed(intake, ballFeed, .61, .75)),false);// Speeds as previously determined
    logitecLeftTrigger.whenReleased(new RetractIntake(intake));
    //Possible tthat the Extend/Retract Intake may be reversed? check this first next meeting

    //Raise or lower the climber hook with up and down D-pad buttons. Move while pressed, stop when released.
    //Allows operator to position the climber hook on the bar.
    logitecDPadUp.whileActiveOnce(new RunClimberHook(climber, 0.5),false);   //Previously used speeds
    logitecDPadDown.whileActiveOnce(new RunClimberHook(climber, -0.25),false);

    //Shift to high or low gear
    leftTrigger.whenActive(new ShiftLow(drivetrain));
    rightTrigger.whenActive(new ShiftHigh(drivetrain));

    //DEAD BAND FOR LOGITECH JOYSTICK CONTROLLERS
    // Lets consider moving these functions to joystick buttons.
    //logitecLeftStick.whenActive(new RunClimber(climber, logitecController.getLeftStickY(), logitecController.getRightStickY()));
    //logitecRightStick.whenActive(new RunClimber(climber, logitecController.getLeftStickY(), logitecController.getRightStickY()));
    //logitecLeftStick.whenActive(new RunClimberLeft(climber, logitecController.getRawAxis(CONSTANTS_OI.XBOX_LEFT_STICK_Y_AXIS)));
    //logitecRightStick.whenActive(new RunClimberRight(climber, logitecController.getRawAxis(CONSTANTS_OI.XBOX_RIGHT_STICK_Y_AXIS)));

    //logitecLeftTrigger.whileHeld(new RunIntake(intake, .61));
    // when intake is done and we want to run it with ball feeder, replace above line with this
    // runIntakeIn.whileHeld(new ParallelCommandGroup(new RunIntake(intake, 1.0), 
    //                                               new RunBallFeed(ballFeed, -1.0)));
    //logitecLeftBumper.whileHeld(new RunIntake(intake, -.61));


    // runIntakeAndBallFeedJoystick.whileHeld(new RunIntakeAndBallFeed(intake, ballFeed, 0.35, 0.75));
    // runShooterJoystick.toggleWhenActive(new RunShooter(shooter, 1.0));
    // runBothFeedersJoystick.whileHeld(new RunBothFeeders(ballFeed));
    // Left and right joysticks. Button below circle pad
    // Left - Default limelight
    // Right - Camera View
  
    // logitecDPadLeft.whileActiveContinuous(new RunColorWheel(colorWheel, 0.5));
    // logitecDPadRight.whileActiveContinuous(new RunColorWheel(colorWheel, -0.5));
    //logitecRightBumper.toggleWhenPressed(new RunShooterPID(shooter, 4900)); //run at max speed

  }
     
  private void configureDashboard(){
    
    //Commented out a bunch of this to declutter the dashboard for competition.
    //I don't think we need most of this anymore

    //Display the name and version number of the code.
    SmartDashboard.putString("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION);

    //SmartDashboard.putData("Score Power Cell", new ShootPowerCell(intake, ballFeed, drivetrain, shooter));
    SmartDashboard.putNumber("Ballfeed Speed", ballFeed.getBallFedVelocity());
    //runShooterAndBallFeed.whenActive(new RunShooterFeed(ballFeed, 0.25), new RunBallFeed(ballFeed, 0.25));


    //SmartDashboard.putData("Extend hood", new ExtendHood(hood));  
    //SmartDashboard.putData("Retract hood", new RetractHood(hood));
   
    //SmartDashboard.putData("Run Intake", new RunIntake(intake, 0.35));

    //SmartDashboard.putData("Run Climber One", new RunClimberLeft(climber, 0.35));
    //SmartDashboard.putData("Stop Climber", new StopClimber(climber));

    //SmartDashboard.putData("Run Climber Two", new RunClimberRight(climber, -0.35));
    //SmartDashboard.putData("Run Climber", new RunClimber(climber, logitecController));

    //SmartDashboard.putData("Extend Intake", new ExtendIntake(intake));
    //SmartDashboard.putData("Retract Intake", new RetractIntake(intake));
    //SmartDashboard.putData("Run BallFeed", new RunBallFeed(ballFeed, -0.50));
    //SmartDashboard.putData("Limelight Aim and Shoot", new LimelightShooter(limelight, drivetrain, shooter, ballFeed));
    //SmartDashboard.putData("Limelight Aim", new LimelightCenter(limelight, drivetrain));
    //SmartDashboard.putData("Run ShooterFeed", new RunShooterFeed(ballFeed, 1.0));

    //SmartDashboard.putData("Sequential Shooter", new SequentialShooter(shooter, ballFeed));
    //SmartDashboard.putData("Fail Safe Auto", new FailSafeAutoWithVelocity(shooter, ballFeed, 1.0, 1.0, 1.0));
    //SmartDashboard.putData("PID Gyro Aim", new PIDGyroAim(drivetrain, 65));
    SmartDashboard.putData("Reset Gyro Senser", new ResetGyro(drivetrain));
    SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroHeading());

    //SmartDashboard.putData("Limelight PID", new LimelightAutoAim(limelight, drivetrain));

    // Interstellar Accuracy Challenge Speed Buttons
    /* Ignore this for now
    SmartDashboard.putData("Shooter Zone 1", new RunShooter(shooter, 0.40));
    SmartDashboard.putData("Shooter Zone 2", new RunShooter(shooter, 0.48)); // hood back 0.34, hood forward 0.48
    SmartDashboard.putData("Shooter Zone 3", new RunShooter(shooter, 0.43)); // hood back 0.43, hood forward ?
    SmartDashboard.putData("Shooter Zone 4", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    SmartDashboard.putData("Shooter Zone 5", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    */
    //SmartDashboard.putNumber("Servo tilt", 0);
    SmartDashboard.putNumber("Shooter RPM", 3400);
    //SmartDashboard.putData("Run Shooter PID", new RunShooterPID(shooter, 3400));
  
    //SmartDashboard.putData("Simply seek powercell", new SimpleSeekPowercell(pixy, drivetrain));

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);
    m_AutoChooser.addOption("OLD - AutoSequentialShooter", m_SequentialShooter);  //Don't know if this works
    m_AutoChooser.addOption("OLD - FailSafeAutoBackwards", m_FailSafeBackward);  //Don't know if this works
    m_AutoChooser.addOption("NEW - ShootTenBackup", m_ShootThenBackup);
    m_AutoChooser.addOption("FINiteRecharge", m_FINiteRechargeCommand);
    

    /* Removing so we don't accidently select during competition.
    m_AutoChooser.addOption("Slalom", m_SlalomAuto);
    m_AutoChooser.addOption("Barrel", m_BarrelAuto);
    m_AutoChooser.addOption("Bounce", m_BounceAuto);
    m_AutoChooser.addOption("Galactic Search", m_GalacticAuto);
    */

    //Put the auto chooser on the dashboard
    SmartDashboard.putData(m_AutoChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_AutoChooser.getSelected();
 
  }
}
