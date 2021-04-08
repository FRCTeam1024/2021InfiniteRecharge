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


  private final Command m_autoCommand = this.getAutonomousCommand();
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
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            7.5); //testing a lower voltage? hopefully that will slow it down so I can figure out what its trying 

    TrajectoryConfig config =
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseConfig = 
      new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                           DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                           // Add kinematics to ensure max speed is actually obeyed
                           .setKinematics(DriveConstants.kDriveKinematics)
                           // Apply the voltage constraint
                           .addConstraint(autoVoltageConstraint);
    reverseConfig.setReversed(true);

    Trajectory Trajectory1 = TrajectoryGenerator.generateTrajectory(
                  /**
                   * 
                   * Slalom Path Trajectory - working, may be tweaked as we get filmed versions.
                  new Pose2d(0.762, 0.762, new Rotation2d(0)),
                  List.of(
                      new Translation2d(2.2, 0.762),
                      new Translation2d(2.2, 2.286),
                      new Translation2d(6.096, 2.286),
                      new Translation2d(6.096, 0.762),
                      new Translation2d(8.0, 0.762),
                      new Translation2d(8.0, 2.286),
                      new Translation2d(7.07, 2.286),
                      new Translation2d(7.07, 0.75),
                      new Translation2d(2.75, 0.75),
                      new Translation2d(2.45, 1.7)
                  ),
                  new Pose2d(1.0, 1.7, new Rotation2d(Constants.PI)),**/
                  /**Barrel Path Trajectory - best 21 seconds**/
                  new Pose2d(0.762, 2.286, new Rotation2d(0)),
                  List.of(
		                new Translation2d(4.572, 2.286),
		                new Translation2d(4.572, 0.9),
		                new Translation2d(3.2, 0.9),
                    new Translation2d(3.2, 2.2),
                    new Translation2d(4.0, 2.2),
                    new Translation2d(6.858, 2.35),
	                	new Translation2d(6.858, 3.4),
                    new Translation2d(5.6, 3.4),
	                	new Translation2d(5.6, 1.4),
	                	new Translation2d(7.7, 1.4),
                    new Translation2d(7.7, 2.286),
                    new Translation2d(6.0, 2.3)
                  ),
                  new Pose2d(1.5, 2.8, new Rotation2d(Constants.PI)),
                  /**Bounce Path Trajectory 
                   * 
                   * I'm working on setting up a trajectory for the bounce path,
                   * but it currently wants to turn 180 degrees at every starred
                   * point. Tried a different constructor for the trajectory,
                   * robot seemed to have an issue with it, will not start at all.
                   * "Robots should not quit, but yours did!"
                   * Insists that there is no robot code
                   * power cycled robot - did not help
                   * returned to barrel path and deployed - no issues there
                  
                  List.of(
                      new Pose2d(0.762, 2.286, new Rotation2d(0)),
                      new Pose2d(1.95, 2.286, new Rotation2d(Constants.PI/4)),
                      new Pose2d(1.95, 3.81, new Rotation2d(Constants.PI/2))
                  ),**/
                  // Pass config
                  config
    );
    /**Trajectory Trajectory2 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(1.95, 3.81, new Rotation2d(Constants.PI/2)),
        new Pose2d(2.286, 2.5, new Rotation2d(3 * Constants.PI/4)),
        new Pose2d(3.048, 2.5, new Rotation2d(Constants.PI/2)),
        new Pose2d(3.048, 0.762, new Rotation2d(3 * Constants.PI/4)),
        new Pose2d(4.572, 0.762, new Rotation2d(-1 * Constants.PI/4)),
        new Pose2d(4.572, 3.81, new Rotation2d(-1 * Constants.PI/2))
      ),
      reverseConfig
    );
    Trajectory Trajectory3 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(4.572, 3.81, new Rotation2d(-1 * Constants.PI/2)),
        new Pose2d(5.1, 0.762, new Rotation2d(-1 * Constants.PI/4)),
        new Pose2d(5.9, 0.762, new Rotation2d(Constants.PI/4)),
        new Pose2d(6.7, 3.7, new Rotation2d(Constants.PI/2))
      ),
      config
    );
    Trajectory Trajectory4 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(6.7, 3.7, new Rotation2d(Constants.PI/2)),
        new Pose2d(6.858, 2.5, new Rotation2d(3 * Constants.PI/4)),
        new Pose2d(7.62, 2.5, new Rotation2d(Constants.PI))
      ),
      reverseConfig
    );**/

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(Trajectory1.getInitialPose());  

    RamseteCommand ramseteCommand = new RamseteCommand(
      Trajectory1,
      drivetrain::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain
    );

    /**RamseteCommand ramseteCommand2 = new RamseteCommand(
      Trajectory2,
      drivetrain::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain
    );
    RamseteCommand ramseteCommand3 = new RamseteCommand(
      Trajectory3,
      drivetrain::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain
    );
    RamseteCommand ramseteCommand4 = new RamseteCommand(
      Trajectory4,
      drivetrain::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain
    );**/

    //return ramseteCommand.andThen(ramseteCommand2).andThen(ramseteCommand3).andThen(ramseteCommand4).andThen(() -> drivetrain.tankDriveVolts(0, 0));
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public void periodic() {
    //SmartDashboard.updateValues();
  }

  public void outputToSmartDashboard() {

  }
}
