/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoSequentialShooter;
import frc.robot.commands.auto.FailSafeAutoBackward;
import frc.robot.commands.auto.FailSafeAutoForward;

import frc.robot.commands.auto.FailSafeAutoWithVelocity;
import frc.robot.commands.auto.LimelightCenter;
import frc.robot.commands.auto.LimelightShooter;
import frc.robot.commands.auto.SequentialShooter;
import frc.robot.oi.CONSTANTS_OI;
import frc.robot.oi.Logitech;
import edu.wpi.first.wpilibj2.command.Command;


import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoMode;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
  private final Sensors sensors = new Sensors();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber  = new Climber();
  // private final ColorWheel colorWheel = new ColorWheel();
  private final BallFeed ballFeed  = new BallFeed();
  private final Limelight limelight = new Limelight();
  private final Pixy2 pixy = new Pixy2();
  private final Hood hood = new Hood();


  public Joystick leftJoystick = new Joystick(1);
  public Joystick rightJoystick = new Joystick(2);
  public Logitech xboxController = new Logitech(0);

  
  // public JoystickButton runShooterAndBallFeed = new JoystickButton(leftJoystick, 6);

  
  // below this done with Marc
  public JoystickButton xboxLeftTrigger = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_LEFT_TRIGGER);
  public JoystickButton xboxLeftBumper = new JoystickButton(xboxController, XboxController.Button.kBumperLeft.value);


  public JoystickButton xboxLeftClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickLeft.value);
  public JoystickButton xboxRightClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickRight.value);
  
  public JoystickButton xboxButtonX = new JoystickButton(xboxController, 1);
  public JoystickButton xboxButtonA = new JoystickButton(xboxController, 2);
  public JoystickButton xboxButtonB = new JoystickButton(xboxController, 3);
  public JoystickButton xboxButtonY = new JoystickButton(xboxController, 4);
  
  public JoystickButton xboxRightBumper = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);
  public JoystickButton xboxRightTrigger = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_RIGHT_TRIGGER);
  
  public Trigger xboxDPadUp = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.UP));
  public Trigger xboxDPadLeft = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.LEFT));
  public Trigger xboxDPadRight = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.RIGHT));
  public Trigger xboxDPadDown = new Trigger( () -> xboxController.getDPadState().equals(Logitech.DPadState.DOWN));

  //JUST FOR MARC
  // public JoystickButton runIntakeAndBallFeedJoystick = new JoystickButton(leftJoystick, 1);
  public JoystickButton shiftHighJoystick = new JoystickButton(leftJoystick, 3);
  public JoystickButton shiftLowJoystick = new JoystickButton(rightJoystick, 4);

  public JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
  public JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);

 

  public JoystickButton switchCamModeDefault = new JoystickButton(leftJoystick, 2);
  public JoystickButton switchCamModeCamera = new JoystickButton(rightJoystick, 2);
  // public JoystickButton runShooterJoystick = new JoystickButton(rightJoystick, 3);
  // public JoystickButton runBothFeedersJoystick = new JoystickButton(rightJoystick, 1);
  public JoystickButton xBoxBackButton = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_BACK_BUTTON);
  public JoystickButton xboxStartButton = new JoystickButton(xboxController, CONSTANTS_OI.XBOX_START_BUTTON);


  // private final Command m_autoCommand = new LimelightCenter(drivetrain);
  // private final Command m_autoCommand = new FailSafeAutoBackward(drivetrain, shooter, ballFeed, 1.0, 1.0, -1.0);
  private final Command m_autoCommand = new AutoSequentialShooter(shooter, ballFeed);

  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drivetrain, leftJoystick, rightJoystick);
  private final DriveClimberDefault driveClimberDefault = new DriveClimberDefault(climber, xboxController);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveWithJoysticks);
    // climber.setDefaultCommand(driveClimberDefault);
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
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		//camera.setResolution(144, 144);

    xboxButtonA.whileHeld(new RunClimberHook(climber, -0.25));
    xboxButtonX.whileHeld(new RunClimberHook(climber, 0.50));
    //xboxButtonY.toggleWhenActive(new RunShooter(shooter, 1)); //Removed in favor of PID control below
    xboxButtonY.toggleWhenActive(new RunShooterPID(shooter, 3400));
    xboxButtonB.whileHeld(new RunBothWinches(climber, 1.0, 1.0));

    shiftHighJoystick.toggleWhenPressed(new ShiftHigh(drivetrain));
    shiftLowJoystick.toggleWhenPressed(new ShiftLow(drivetrain));

    leftTrigger.whenPressed(new LimelightCenter(limelight, drivetrain));
    rightTrigger.whenPressed(new LimelightShooter(limelight, drivetrain, shooter, ballFeed));

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
    xBoxBackButton.whileHeld(new RunBallFeed(ballFeed, -0.75));

    new LimelightLED(limelight, 0); // Disable Limelight LEDs
    
    SmartDashboard.putData("Score Power Cell", new ShootPowerCell(intake, ballFeed, drivetrain, shooter));
    SmartDashboard.putNumber("Ballfeed Speed", ballFeed.getBallFedVelocity());
    //runShooterAndBallFeed.whenActive(new RunShooterFeed(ballFeed, 0.25), new RunBallFeed(ballFeed, 0.25));
    
    // Shooter Info - Ignoring this for now
    //SmartDashboard.putNumber("Desired Shooter Speed", 1.0);
    //SmartDashboard.putData("Set Shooters to Desired Speed", new AdjustShooterSpeed(shooter));
    // SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());

    // Shooter Hood
    SmartDashboard.putData("Extend hood", new ExtendHood(hood));  // I think the hood needs its own subsystem so it can be changed while running the shooter
    SmartDashboard.putData("Retract hood", new RetractHood(hood));
   
    SmartDashboard.putData(drivetrain);  
    SmartDashboard.putData("Run Intake", new RunIntake(intake, 0.35));

    // Don't need these, have this elsewhere now
    // SmartDashboard.putData("Run Shooter", new RunShooter(shooter, 1.0));
    // SmartDashboard.putNumber("Velocity", shooter.shooterEncoderOne.getVelocity());

    SmartDashboard.putData("Run Climber One", new RunClimberLeft(climber, 0.35));
    SmartDashboard.putData("Stop Climber", new StopClimber(climber));

    SmartDashboard.putData("Run Climber Two", new RunClimberRight(climber, -0.35));
    SmartDashboard.putData("Run Climber", new RunClimber(climber, -0.35, 0.35));
    Shuffleboard.getTab("Climber").add("Run Climber", new RunClimber(climber, 0.35, -0.35));


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
    SmartDashboard.putNumber("Gyro Angle", drivetrain.ahrs.getAngle());

    SmartDashboard.putData("Limelight PID", new LimelightAutoAim(limelight, drivetrain));
    //SmartDashboard.putNumber("LkFF", limelight.getPIDController().getP());

    // Interstellar Accuracy Challenge Speed Buttons
    /* Ignore this for now
    SmartDashboard.putData("Shooter Zone 1", new RunShooter(shooter, 0.40));
    SmartDashboard.putData("Shooter Zone 2", new RunShooter(shooter, 0.48)); // hood back 0.34, hood forward 0.48
    SmartDashboard.putData("Shooter Zone 3", new RunShooter(shooter, 0.43)); // hood back 0.43, hood forward ?
    SmartDashboard.putData("Shooter Zone 4", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    SmartDashboard.putData("Shooter Zone 5", new RunShooter(shooter, 0.40)); // hood back ?, hood forward ?
    */

    // I think this will allow a speed value to be provided and the shooter to be run at that speed
    SmartDashboard.putNumber("Shooter RPM", 3400);
    SmartDashboard.putData("Run Shooter PID", new RunShooterPID(shooter, 3400));
    //Shuffleboard.getTab("Shooter").add("Run Shooter PID", new RunShooterPID(shooter, 0.1));
  
    SmartDashboard.putData("Enable LEDs", new LimelightLED(limelight, 1));
    SmartDashboard.putData("Disable LEDs", new LimelightLED(limelight, 0));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
    // return limelightCenterPID;
  }

  public void periodic() {
    outputToSmartDashboard();

  }

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("Yaw", sensors.getHeading());
    SmartDashboard.putData("Reset Gyro", new InstantCommand(sensors::resetGyro));
  }
}
