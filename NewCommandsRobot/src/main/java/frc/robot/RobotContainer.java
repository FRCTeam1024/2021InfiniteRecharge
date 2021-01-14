/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.TurnGyroRight;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAnglePID;
import frc.robot.commands.auto.LimelightCenter;
import frc.robot.commands.auto.LimelightCenterLoggable;
import frc.robot.commands.auto.LimelightCenterPID;
import frc.robot.commands.auto.LimelightCenterSequential;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDDrivetrain;
import frc.robot.subsystems.Sensors;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final PIDDrivetrain drivetrainPID = new PIDDrivetrain();
  private final Sensors sensors = new Sensors();

  public Joystick leftJoystick = new Joystick(2);
  public Joystick rightJoystick = new Joystick(0);

  public JoystickButton autoCenter = new JoystickButton(leftJoystick, 2);
  public JoystickButton runShooter = new JoystickButton(leftJoystick, 3);

  private final Command m_autoCommand = new LimelightCenterSequential(drivetrain);
  // private final Command limelightCenterPID = new LimelightCenterPID(drivetrainPID);
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
    SmartDashboard.putData("Turn Command", new TurnToAnglePID(90, drivetrain, sensors));
    autoCenter.whileHeld(m_autoCommand);
    SmartDashboard.putData("Limelight Center PID class", new InstantCommand(drivetrainPID::enable, drivetrainPID));

    SmartDashboard.putData("Limelight Center Loggable", new LimelightCenterLoggable(drivetrain));

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry xOffset = limelight.getEntry("tx");
    
    PIDController pidController = drivetrainPID.getController();// new PIDController(PIDDrivetrain.Kp, PIDDrivetrain.Ki, PIDDrivetrain.Kd);
    // pidController.setName("PID Drivetrain inline"); // this was so it could be identified on Shuffleboard
    pidController.setTolerance(0.5); // this is not used automatically; think it is used when calling pidController.atSetpoint()

    SmartDashboard.putNumber("Kp", pidController.getP());
    SmartDashboard.putNumber("Ki", pidController.getI());
    SmartDashboard.putNumber("Kd", pidController.getD());
    SmartDashboard.putNumber("xOffset", xOffset.getDouble(0.0));
    
    SmartDashboard.putData(drivetrain);
    
    SmartDashboard.putData("Limelight PID Inline", new PIDCommand(
        pidController, 
        // this is getMeasurement
        () -> { 
          System.out.println("inline xOffset : " + xOffset.getDouble(0.0));
          return xOffset.getDouble(0.0);
        }, 
        // this is the setpoint
        0.0, 
        // this is useOutput
        output -> {
          System.out.println("output : " + output);
          double minPower = .2;
          double maxPower = 0.95;
          double tolerance = .2;
          double finalPower = minPower + Math.abs(output);
          if(finalPower < minPower) {
            finalPower = minPower;
          }
          if(finalPower > maxPower) {
            finalPower = maxPower;
          }
          // if(Math.abs(xOffset.getDouble(0.0)) <= tolerance) {
          //   if(Math.abs(xOffset.getDouble(0.0)) < tolerance +.05 ) {
          //     finalPower = 0;
          //   }
          // }
          System.out.println("finalPower : " + finalPower);
          if(output < 0) { // we want to turn right
            drivetrain.drive(finalPower, -finalPower);
          } else {
            drivetrain.drive(-finalPower, finalPower);
          }
          
        }, 
        // requirements
        drivetrain)
        // decorators
          .beforeStarting(() -> {
            double newKP = SmartDashboard.getNumber("kP", pidController.getP());
            System.out.println("new KP : " + newKP);
            pidController.setP(newKP);
            double newKD = SmartDashboard.getNumber("kD", pidController.getD());
            System.out.println("new KD : " + newKD);
            pidController.setD(newKD);
          })
          .withInterrupt(pidController::atSetpoint) // PIDController.setTolerance() is checked here
        ); // close of putData

    SmartDashboard.putData("Move Command", new StartEndCommand(
                              // start driving forward
                              () -> drivetrain.driveForward(.25),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1));

    SmartDashboard.putData("Turn Right Command", new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.turnRight(.50),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1));
                            
    
    SmartDashboard.putData("Turn Left Command", new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.turnLeft(.50),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                              .withTimeout(1));
    SmartDashboard.putData("Pivot Turn Right Command", new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.pivotTurnRight(0.5, 0.5),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1));
    SmartDashboard.putData("Pivot Turn Left Command", new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.pivotTurnLeft(0.5, 0.5),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1));
                            
   /* SmartDashboard.putData("Pivot Turn Right Angle Command", new StartEndCommand(
                              () -> drivetrain.ahrs.reset(),
                              () -> drivetrain.pivotTurnRightAngle(0.5, 0.5, 90),
                               drivetrain)); */
    // example of an in-line, sequential command
    // this is another way to do a CommandGroup of sequential commands
    SmartDashboard.putData("turn right gyro not inline", new TurnGyroRight(drivetrain, 0.25, 0.25, 90)); //turns and stops but is not accurate, has been overestimating anywhere from 5-30 degrees
    SmartDashboard.putData("MoveTurn Right Command", new StartEndCommand(
                              // start driving forward
                              () -> drivetrain.driveForward(.30),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1)
                            .andThen(new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.turnRight(.50),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1)));

    SmartDashboard.putData("MoveTurn Left Command", new StartEndCommand(
                              // start driving forward
                              () -> drivetrain.driveForward(.30),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1)
                            .andThen(new StartEndCommand(
                              // start doing a turn
                              () -> drivetrain.turnLeft(.50),
                              // stop driving
                              () -> drivetrain.stop(),
                              // subsystem requirements here 
                              drivetrain)
                              // this is a command decorator; a convenience method
                            .withTimeout(1)));
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
