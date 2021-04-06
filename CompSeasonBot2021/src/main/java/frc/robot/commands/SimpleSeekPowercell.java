// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoTurnHeading;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;

/**
 * This command is used to align the robot to the nearest powercell.
 */
public class SimpleSeekPowercell extends CommandBase {
  // Subsystems
  private final PixyCam pixy;
  private final Drivetrain drivetrain;
  // Data
  private final double errorThreshold = 0.5; // Must be within 0.5 pixels
  private final double alignSpeed = 0.275; // Speed to drive when aligning to target
  private final double findSpeed = 0.5; // Speed to drive when looking for a target
  private double powercellX, xError; // Saving the powercell X position and the distance from our goal
  // Booleans for checking if commands is finished
  private boolean powercellDetected, isFinished;

  /**
   * Aligns the robot to the nearest powercell, according to the Pixy2 camera.
   * @param pixySubsystem
   * @param drivetrainSubsystem
   */
  public SimpleSeekPowercell(PixyCam pixySubsystem, Drivetrain drivetrainSubsystem) {
    addRequirements(pixySubsystem, drivetrainSubsystem);
    pixy = pixySubsystem;
    drivetrain = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset finished check booleans
    powercellDetected = false;
    isFinished = false;

    drivetrain.shiftLow(); // Shift into low gear
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If no powercell is in our line of sight
    if(pixy.getXOffset() == -1) {

      // NOTE: This could potentially be used for later in case
      //        the current implementation does not suit our needs.

      /*double startHeading = drivetrain.getGyroHeading();
      if(drivetrain.getGyroHeading() > 0) {
        while(drivetrain.getGyroHeading() > startHeading - 90) {
          drivetrain.drive(-speed, speed);
        }
      } else {
        while(drivetrain.getGyroHeading() < startHeading + 90) {
          drivetrain.drive(speed, -speed);
        }
      }*/

      // If we're to the right of our starting angle, turn left 90 degrees, else, right 90.
      double desiredHeading = drivetrain.getGyroHeading() >= 0 ? drivetrain.getGyroHeading() - 90 : drivetrain.getGyroHeading() + 90;
      // If we need to turn right 90 degrees
      if(drivetrain.getGyroHeading() < desiredHeading) {
        // While we are still to the left of our goal
        while(drivetrain.getGyroHeading() < desiredHeading) {
          drivetrain.drive(findSpeed, -findSpeed); // Turn right
        }
      } else { // If we need to turn left 90 degrees
        // While we are still to the right of our goal
        while(drivetrain.getGyroHeading() > desiredHeading) {
          drivetrain.drive(-findSpeed, findSpeed); // Turn left
        }
      }
    // If we see a powercell
    } else {
      powercellX = pixy.getXOffset(); // Get the horizontal alignment to the powercell
      powercellDetected = true; // Record that we found a powercell
      xError = powercellX - Constants.PixyConstants.HALF_WIDTH; // Get the offset between the robot and the target
      if(Math.abs(xError) <= errorThreshold) { // If the robot is within the target offset threshold
        isFinished = true; // We're done
      } else if(xError > 0) { // If the robot is to the left of the target
        drivetrain.drive(alignSpeed, 0); // Turn right
      } else if(xError < 0) { // If the robot is to the right of the target
        drivetrain.drive(0, alignSpeed); // Turn left
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0); // Shut off drivetrain motors when command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished && powercellDetected; // Finish once we've seen a powercell and we're aligned with it
  }
}
