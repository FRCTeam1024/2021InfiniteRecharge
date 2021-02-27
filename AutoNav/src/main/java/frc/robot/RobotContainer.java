/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.oi.MustangController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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


  public Joystick leftJoystick = new Joystick(0);
  public Joystick rightJoystick = new Joystick(1);
  public MustangController xboxController = new MustangController(2);

  
  public JoystickButton autoCenter = new JoystickButton(leftJoystick, 2);
  public JoystickButton runShooter = new JoystickButton(leftJoystick, 7);
  public JoystickButton scorePowerCell = new JoystickButton(leftJoystick, 8);
  public JoystickButton runIntake = new JoystickButton(leftJoystick, 9);
  public JoystickButton runShooterAndBallFeed = new JoystickButton(leftJoystick, 6);

  // JoystickButton leftTriggerButton = new JoystickButton(xboxController, (xboxController) -> xboxController.getTriggerAxis(Hand.kLeft) >= 0.5);

  public JoystickButton runLeftClimberButtonDown = new JoystickButton(xboxController, XboxController.Button.kX.value);
  public JoystickButton runLeftClimberButtonUp = new JoystickButton(xboxController, XboxController.Button.kY.value);
  
  // below this done with Marc
  public JoystickButton runLeftClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickLeft.value);
  public JoystickButton runRightClimberStick = new JoystickButton(xboxController, XboxController.Button.kStickRight.value);
  
  public JoystickButton runClimberHookDown = new JoystickButton(xboxController, XboxController.Button.kA.value);
  public JoystickButton runClimberHookUp = new JoystickButton(xboxController, XboxController.Button.kB.value);
  
  public JoystickButton runShooterWheel = new JoystickButton(xboxController, XboxController.Button.kBumperLeft.value);
  public JoystickButton runShooterFeedWheel = new JoystickButton(xboxController, XboxController.Button.kBumperRight.value);
  
  // public JoystickButton intakeIn = new JoystickButton(xboxController, xboxController.getDPadState());
  public Trigger intakeUp = new Trigger( () -> xboxController.getDPadState().equals(MustangController.DPadState.UP));
  public Trigger intakeDown = new Trigger( () -> xboxController.getDPadState().equals(MustangController.DPadState.DOWN));

  public Trigger shooterTrigger = new Trigger( () -> xboxController.getLeftTriggerAxis() > 0.50 );


  private final Command m_autoCommand = new LimelightCenter(drivetrain);
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drivetrain, leftJoystick, rightJoystick);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(driveWithJoysticks);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // autoCenter.whileHeld(m_autoCommand);
    
    // NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry xOffset = limelight.getEntry("tx");
    
    SmartDashboard.putData("Drive", new BasicDriveCommand(drivetrain));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return limelightCenterPID;
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                          DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                  // Start at the origin facing the +X direction
                  new Pose2d(0, 0, new Rotation2d(0)),
                  // Pass through these two interior waypoints, making an 's' curve path
                  List.of(
                      new Translation2d(1, 1),
                      new Translation2d(2, -1)
                  ),
                  // End 3 meters straight ahead of where we started, facing forward
                  new Pose2d(3, 0, new Rotation2d(0)),
                  // Pass config
                  config
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());  //idk what m_robotDrive is, need to look at that some next time

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_robotDrive::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_robotDrive::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_robotDrive::tankDriveVolts,
      m_robotDrive
  );

    return m_autoCommand;
  }

  public void periodic() {
    SmartDashboard.updateValues();
  }

  public void outputToSmartDashboard() {
  }
}
