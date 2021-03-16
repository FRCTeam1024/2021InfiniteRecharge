// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoTurnHeading;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;

/**
 * This command is used to align the robot to the nearest powercell.
 */
public class SimpleSeekPowercell extends CommandBase {
  private final PixyCam pixy;
  private final Drivetrain drivetrain;
  private final int errorThreshold = 1; // Must be within 15 (out of 360) pixels.
  private final double speed = 0.275; // Speed to drive the robot.
  private boolean powercellDetected;
  private double powercellX;
  private double xError;
  boolean isFinished;

  private boolean needsToTurn;
  private double startHeading, currentHeading, desiredHeading, headingThreshold;

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
    powercellDetected = false;
    isFinished = false;
    needsToTurn = false;
    startHeading = drivetrain.getGyroHeading(); // Get our initial angle for turning when nothing is found
    headingThreshold = 1;
    drivetrain.shiftLow(); // Shift into low gear
    //drivetrain.shiftHi();

    SmartDashboard.putBoolean("Seeking powercell", true);

    if(pixy.getXOffset() == -1) { // If no powercell is in front of us
      needsToTurn = true;
      if(drivetrain.getGyroHeading() > 0) { // If we're right of our initial heading
        desiredHeading = drivetrain.getGyroHeading() - 90; // Set heading to 90 degrees left
        //desiredHeading = -90;
      } else { // If we're left of our initial heading
        desiredHeading = drivetrain.getGyroHeading() + 90; // Set heading to 90 degrees right
        //desiredHeading = 90;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(needsToTurn) { // If we need to turn 90 degrees
      currentHeading = drivetrain.getGyroHeading(); // Get the current heading
      System.out.println(currentHeading);
      if(currentHeading < desiredHeading - headingThreshold) { // If we're left of our desired heading
        drivetrain.drive(0.5, -0.5); // Turn right
      } else if(currentHeading > desiredHeading + headingThreshold) { // If we're right of our desired heading
        drivetrain.drive(-0.5, 0.5); // Turn left
      } else { // If we're within our heading thresholds
        drivetrain.drive(0, 0); // Stop driving
        needsToTurn = false;
      }
    } else { // If we didn't need to turn or have already
      powercellX = pixy.getXOffset(); // Get the horizontal alignment to the powercell
      // @TODO: Turn left or right 90 degrees if no powercell was found initially
      if(powercellX == -1) { // If we don't find a powercell
        if(startHeading > 0) { // And we're too far to the right
          drivetrain.drive(-0.5, 0.5); // Turn left until we see a powercell
        } else if(startHeading < 0) { // Or we're too far to the left
          drivetrain.drive(0.5, -0.5); // Turn right until we see a powercell
        }
      } else { // If we do see a powercell
        powercellDetected = true;
        xError = powercellX - Constants.PixyConstants.HALF_WIDTH; // Get the offset between the robot and the target
        if(Math.abs(xError) <= errorThreshold) { // If the robot is within the target offset threshold
          isFinished = true; // We're done
        } else if(xError > 0) { // If the robot is to the left of the target
          drivetrain.drive(speed, -speed); // Turn right
        } else if(xError < 0) { // If the robot is to the right of the target
          drivetrain.drive(-speed, speed); // Turn left
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Seeking powercell", false);
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished && powercellDetected; // Finish once we've seen a powercell and we're aligned with it
  }
}
