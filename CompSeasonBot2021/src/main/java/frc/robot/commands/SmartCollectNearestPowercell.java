// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoTurnHeading;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PixyCam;

/**
 * This command will drive forward until the intake collects a powercell
 * @TODO: Align the robot with the powercell as it drives? Implement SimpleSeekPowercell for this
 */
public class SmartCollectNearestPowercell extends CommandBase {
  private final Intake intake;
  private final BallFeed ballfeed;
  private final Drivetrain drivetrain;
  private final PixyCam pixy;

  double currentXOffset, currentYOffset;
  private double speed = 0.5; // feet per second

  private boolean hasPowercell;

  /**
   * This command collects a powercell directly in front of the robot.
   * 
   * @param intakeSubsystem
   * @param ballfeedSubsystem
   * @param drivetrainSubsystem
   * @param pixySubsystem
   */
  public SmartCollectNearestPowercell(Intake intakeSubsystem, BallFeed ballfeedSubsystem,
      Drivetrain drivetrainSubsystem, PixyCam pixySubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    ballfeed = ballfeedSubsystem;
    drivetrain = drivetrainSubsystem;
    pixy = pixySubsystem;
    addRequirements(intake, ballfeed, drivetrain, pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasPowercell = false;
    drivetrain.shiftLow(); // Shift into low gear

    if(pixy.getXOffset() == -1) {
      double desiredHeading;
      double headingThreshold = 5;

      if(drivetrain.getGyroHeading() >= 0) {
        desiredHeading = -90;
      } else {
        desiredHeading = 90;
      }

      CommandBase turnCommand = new AutoTurnHeading(drivetrain, desiredHeading);
      new WaitUntilCommand(turnCommand::isFinished);
      /*do {
        new Turn
      } while(drivetrain.getGyroHeading() < desiredHeading - headingThreshold || drivetrain.getGyroHeading() > desiredHeading + headingThreshold);
      */// This while loop triggers if we are too far to the left or right of our desired heading (not aligned).
    }

    intake.runIntake(0.35); // Run the intake
    ballfeed.runBallFeedMotor(0.75); // Run the ballfeed motor
    //drivetrain.driveSpeed(speed, false); // Drive forward at 5ft/sec in low gear 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentXOffset = pixy.getXOffset();
    currentYOffset = pixy.getYOffset();
    //double xError = Math.abs(Constants.PixyConstants.HALF_WIDTH - currentXOffset);
    double slowCoefficient = 0.3; // Slows the speed of one side of wheels to adjust itself.
                                  // This variable may potentially need to adjust itself based on the distance from the target (error).
    System.out.println(currentXOffset);
    if(currentXOffset < Constants.PixyConstants.HALF_WIDTH) {
      drivetrain.drive(speed, speed * slowCoefficient); // Slow the right wheels to skew right.
    } else if(currentXOffset > Constants.PixyConstants.HALF_WIDTH) {
      drivetrain.drive(speed * slowCoefficient, speed); // Slow the left wheels to skew left.
    } else {
      drivetrain.drive(speed, speed);
    }

    if(currentYOffset > 185 || currentYOffset == -1) { // If the powercell is within the relative area of our intake
      Timer.delay(1); // Wait 1 second for it to be collected
      hasPowercell = true; // The robot has the powercell, we're finished
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If this command is ended or interrupted, stop all motors.
    intake.runIntake(0);
    ballfeed.runBallFeedMotor(0);
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPowercell; // If we have a powercell, we're done.
  }
}
