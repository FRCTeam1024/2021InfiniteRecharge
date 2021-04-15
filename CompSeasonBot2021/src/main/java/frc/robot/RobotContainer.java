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

import frc.robot.commands.auto.FailSafeAutoWithVelocity;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.LimelightCenter;
import frc.robot.commands.auto.LimelightShooter;
import frc.robot.commands.auto.SequentialShooter;
import frc.robot.commands.auto.SlalomPathArc;
import frc.robot.commands.auto.BouncePathArc;
import frc.robot.commands.auto.BarrelPathArc;
import frc.robot.oi.CONSTANTS_OI;
import frc.robot.oi.Logitech;
import edu.wpi.first.wpilibj2.command.Command;

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
  private final Logitech xboxController = new Logitech(0);

  //Define various autos to select from
  private final Command m_SlalomAuto = new SlalomPathArc(drivetrain);
  private final Command m_BarrelAuto = new BarrelPathArc(drivetrain);
  private final Command m_BounceAuto = new BouncePathArc(drivetrain);
  // private final Command m_autoCommand = new LimelightCenter(drivetrain);
  // private final Command m_autoCommand = new FailSafeAutoBackward(drivetrain, shooter, ballFeed, 1.0, 1.0, -1.0);
  //private final Command m_autoCommand = new AutoSequentialShooter(shooter, ballFeed);
  private final Command m_GalacticAuto = new GalacticSearch(drivetrain, intake, pixy, ballFeed);

  //Create a chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drivetrain, leftJoystick, rightJoystick);
  //private final AnglePixy anglePixy = new AnglePixy(pixy);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Put some buttons on the dashboard
    configureDashboard();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithJoysticks);
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
     // public JoystickButton runShooterAndBallFeed = new JoystickButton(leftJoystick, 6);
  
    // Buttons on each controller are defined here...
    final JoystickButton xboxLeftTrigger = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_LEFT_TRIGGER);
    final JoystickButton xboxLeftBumper = new JoystickButton(xboxController, XboxController.Button.kBumperLeft.value);


    final JoystickButton xboxLeftClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickLeft.value);
    final JoystickButton xboxRightClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickRight.value);
    
    final JoystickButton xboxButtonX = new JoystickButton(xboxController, 1);
    final JoystickButton xboxButtonA = new JoystickButton(xboxController, 2);
    final JoystickButton xboxButtonB = new JoystickButton(xboxController, 3);
    final JoystickButton xboxButtonY = new JoystickButton(xboxController, 4);
    
    final JoystickButton xboxRightBumper = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);
    final JoystickButton xboxRightTrigger = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_RIGHT_TRIGGER);
    
    final Trigger xboxDPadUp = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.UP));
    final Trigger xboxDPadLeft = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.LEFT));
    final Trigger xboxDPadRight = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.RIGHT));
    final Trigger xboxDPadDown = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.DOWN));

    final JoystickButton xboxBackButton = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_BACK_BUTTON);
    final JoystickButton xboxStartButton = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_START_BUTTON);

    //JUST FOR MARC
    // public JoystickButton runIntakeAndBallFeedJoystick = new JoystickButton(leftJoystick, 1);
    final JoystickButton leftButton3 = new JoystickButton(leftJoystick, 3);
    final JoystickButton leftButton4 = new JoystickButton(rightJoystick, 4);

    final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);

    final JoystickButton button5 = new JoystickButton(rightJoystick, 5);
    final JoystickButton button6 = new JoystickButton(rightJoystick, 6);

    final JoystickButton button7 = new JoystickButton(rightJoystick, 7);

    final JoystickButton button13 = new JoystickButton(rightJoystick, 13);
    final JoystickButton button12 = new JoystickButton(rightJoystick, 12);

    final JoystickButton switchCamModeDefault = new JoystickButton(leftJoystick, 2);
    final JoystickButton switchCamModeCamera = new JoystickButton(rightJoystick, 2);
    // public JoystickButton runShooterJoystick = new JoystickButton(rightJoystick, 3);
    // public JoystickButton runBothFeedersJoystick = new JoystickButton(rightJoystick, 1);
 

    // Linking buttons to commands here...
    /*
    xboxButtonA.whileHeld(new RunClimberHook(climber, -0.25));
    xboxButtonX.whileHeld(new RunClimberHook(climber, 0.50));
    xboxButtonY.toggleWhenActive(new RunShooterPID(shooter, 3400));
    xboxButtonB.whileHeld(new RunBothWinches(climber, 1.0, 1.0));
    */

    xboxButtonA.toggleWhenActive(new RunShooterPID(shooter, 4900));
    xboxButtonX.toggleWhenActive(new RunShooterPID(shooter, 2400));
    xboxButtonY.toggleWhenActive(new RunShooterPID(shooter, 2600));
    xboxButtonB.whileHeld(new RunBothWinches(climber, 1.0, 1.0));
    

    leftButton3.toggleWhenPressed(new ShiftHigh(drivetrain));
    leftButton4.toggleWhenPressed(new ShiftLow(drivetrain));

    leftTrigger.whenPressed(new LimelightCenter(limelight, drivetrain));
    rightTrigger.whenPressed(new LimelightShooter(limelight, drivetrain, shooter, ballFeed));

    button5.toggleWhenPressed(new ExtendHood(hood));
    button6.toggleWhenPressed(new RetractHood(hood));

    button7.toggleWhenPressed(new RunShooter(shooter, 0.75));

    button13.whenPressed(new LimelightLED(limelight, 1));
    button12.whenPressed(new LimelightLED(limelight, 0));

    //DEAD BAND FOR LOGITECH JOYSTICK CONTROLLERS
    if(xboxController.getLeftStickY() > 0.2 || xboxController.getLeftStickY() < 0.2){
      xboxLeftClimberStick.whenActive(new RunClimberLeft(climber, xboxController.getRawAxis(CONSTANTS_OI.XBOX_LEFT_STICK_Y_AXIS)));
    }
    if(xboxController.getRightStickY() > 0.2 || xboxController.getRightStickY() < 0.2){
      xboxLeftClimberStick.whenActive(new RunClimberRight(climber, xboxController.getRawAxis(CONSTANTS_OI.XBOX_LEFT_STICK_Y_AXIS)));
    }
    //xboxLeftClimberStick.whenActive(new RunClimberLeft(climber, xboxController.getRawAxis(CONSTANTS_OI.XBOX_LEFT_STICK_Y_AXIS)));
    //  xboxRightClimberStick.whenActive(new RunClimberRight(climber, xboxController.getRawAxis(CONSTANTS_OI.XBOX_RIGHT_STICK_Y_AXIS)));

    //xboxLeftTrigger.whileHeld(new RunIntake(intake, .61));
    // when intake is done and we want to run it with ball feeder, replace above line with this
    // runIntakeIn.whileHeld(new ParallelCommandGroup(new RunIntake(intake, 1.0), 
    //                                               new RunBallFeed(ballFeed, -1.0)));
    xboxLeftTrigger.whileHeld(new RunIntakeAndBallFeed(intake, ballFeed, 0.35, 0.75));
    //xboxLeftBumper.whileHeld(new RunIntake(intake, -.61));
    xboxLeftBumper.whileHeld(new RunIntakeAndBallFeedAndShooterFeed(intake, ballFeed, -0.35, -0.75, 1.0));

    // runIntakeAndBallFeedJoystick.whileHeld(new RunIntakeAndBallFeed(intake, ballFeed, 0.35, 0.75));
    // runShooterJoystick.toggleWhenActive(new RunShooter(shooter, 1.0));
    // runBothFeedersJoystick.whileHeld(new RunBothFeeders(ballFeed));
    // Left and right joysticks. Button below circle pad
    // Left - Default limelight
    // Right - Camera View
    switchCamModeDefault.toggleWhenPressed(new SwitchCamMode(0));
    switchCamModeCamera.toggleWhenPressed(new SwitchCamMode(1));
    xboxDPadUp.toggleWhenActive(new ExtendIntake(intake));
    xboxDPadDown.toggleWhenActive(new RetractIntake(intake));
  
    // xboxDPadLeft.whileActiveContinuous(new RunColorWheel(colorWheel, 0.5));
    // xboxDPadRight.whileActiveContinuous(new RunColorWheel(colorWheel, -0.5));
    xboxRightBumper.toggleWhenPressed(new RunShooterPID(shooter, 4900)); //run at max speed
    xboxRightTrigger.whileHeld(new RunShooterFeed(ballFeed, -1.0));

    xboxStartButton.whileHeld(new RunBallFeed(ballFeed, 0.75));

    xboxBackButton.whileHeld(new RunBallFeed(ballFeed, -0.75));
  }

  private void configureDashboard(){
    
    SmartDashboard.putData("Score Power Cell", new ShootPowerCell(intake, ballFeed, drivetrain, shooter));
    SmartDashboard.putNumber("Ballfeed Speed", ballFeed.getBallFedVelocity());
    //runShooterAndBallFeed.whenActive(new RunShooterFeed(ballFeed, 0.25), new RunBallFeed(ballFeed, 0.25));


    SmartDashboard.putData("Extend hood", new ExtendHood(hood));  
    SmartDashboard.putData("Retract hood", new RetractHood(hood));
   
    SmartDashboard.putData(drivetrain);  
    SmartDashboard.putData("Run Intake", new RunIntake(intake, 0.35));

    SmartDashboard.putData("Run Climber One", new RunClimberLeft(climber, 0.35));
    SmartDashboard.putData("Stop Climber", new StopClimber(climber));

    SmartDashboard.putData("Run Climber Two", new RunClimberRight(climber, -0.35));
    SmartDashboard.putData("Run Climber", new RunClimber(climber, -0.35, 0.35));

    SmartDashboard.putData("Extend Intake", new ExtendIntake(intake));
    SmartDashboard.putData("Retract Intake", new RetractIntake(intake));
    SmartDashboard.putData("Run BallFeed", new RunBallFeed(ballFeed, -0.50));
    SmartDashboard.putData("Limelight Aim and Shoot", new LimelightShooter(limelight, drivetrain, shooter, ballFeed));
    SmartDashboard.putData("Limelight Aim", new LimelightCenter(limelight, drivetrain));
    SmartDashboard.putData("Run ShooterFeed", new RunShooterFeed(ballFeed, 1.0));
    SmartDashboard.putData("Drive", new BasicDriveCommand(drivetrain));
    SmartDashboard.putData("Sequential Shooter", new SequentialShooter(shooter, ballFeed));
    SmartDashboard.putData("Fail Safe Auto", new FailSafeAutoWithVelocity(shooter, ballFeed, 1.0, 1.0, 1.0));
    SmartDashboard.putData("PID Gyro Aim", new PIDGyroAim(drivetrain, 65));
    SmartDashboard.putData("Reset Gyro Senser", new ResetGyro(drivetrain));
    SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroHeading());

    SmartDashboard.putData("Limelight PID", new LimelightAutoAim(limelight, drivetrain));

    // Interstellar Accuracy Challenge Speed Buttons
    /* Ignore this for now
    SmartDashboard.putData("Shooter Zone 1", new RunShooter(shooter, 0.40));
    SmartDashboard.putData("Shooter Zone 2", new RunShooter(shooter, 0.48)); // hood back 0.34, hood forward 0.48
    SmartDashboard.putData("Shooter Zone 3", new RunShooter(shooter, 0.43)); // hood back 0.43, hood forward ?
    SmartDashboard.putData("Shooter Zone 4", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    SmartDashboard.putData("Shooter Zone 5", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    */
    SmartDashboard.putNumber("Servo tilt", 0);
    SmartDashboard.putNumber("Shooter RPM", 3400);
    SmartDashboard.putData("Run Shooter PID", new RunShooterPID(shooter, 3400));
  
    //SmartDashboard.putData("Simply seek powercell", new SimpleSeekPowercell(pixy, drivetrain));

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);
    m_AutoChooser.addOption("Slalom", m_SlalomAuto);
    m_AutoChooser.addOption("Barrel", m_BarrelAuto);
    m_AutoChooser.addOption("Bounce", m_BounceAuto);
    m_AutoChooser.addOption("Galactic Search", m_GalacticAuto);

    //Put the auto chooser on the dashboard
    SmartDashboard.putData(m_AutoChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new GalacticSearch(drivetrain, intake, pixy, ballFeed);
    //return m_AutoChooser.getSelected();
    // return limelightCenterPID;
  }
}
