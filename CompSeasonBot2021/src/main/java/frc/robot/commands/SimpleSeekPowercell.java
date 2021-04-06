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
  private final PixyCam pixy;
  private final Drivetrain drivetrain;
  private final double errorThreshold = 0.5; // Must be within 15 (out of 360) pixels.
  private final double alignSpeed = 0.275; // Speed to drive when aligning to target
  private final double findSpeed = 0.5; // Speed to drive when looking for a target
  private boolean powercellDetected;
  private double powercellX, gyroHeading;
  private double xError;
  private double driveSpeed;
  private boolean isFinished;

  private boolean hasFirstCell;

  /**
   * Aligns the robot to the nearest powercell, according to the Pixy2 camera.
   * @param pixySubsystem
   * @param drivetrainSubsystem
   */
  public SimpleSeekPowercell(PixyCam pixySubsystem, Drivetrain drivetrainSubsystem) {
    addRequirements(pixySubsystem, drivetrainSubsystem);
    pixy = pixySubsystem;
    drivetrain = drivetrainSubsystem;
    hasFirstCell = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    powercellDetected = false;
    isFinished = false;
    //driveSpeed = 0.5;

    drivetrain.shiftLow(); // Shift into low gear
    //drivetrain.shiftHi();

    SmartDashboard.putBoolean("Has gotten first cell", hasFirstCell);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //powercellX = pixy.getXOffset();
    //gyroHeading = drivetrain.getGyroHeading();

    if(pixy.getXOffset() == -1) {
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

      // If we're to the right of our starting angle, turn to -90 degrees, else, to 90.
      double desiredHeading = drivetrain.getGyroHeading() >= 0 ? drivetrain.getGyroHeading() - 90 : drivetrain.getGyroHeading() + 90;
      //new AutoTurnHeading(drivetrain, desiredHeading);
      
      /*// If we need to go to the left, set direction to -1, else, to 1;
      double turnDirection = desiredHeading < 0 ? -1 : 1;

      // While we haven't passed our heading goal
      while(Math.abs(drivetrain.getGyroHeading()) < Math.abs(desiredHeading)) {
        // Turn in accordance to our direction
        drivetrain.drive(speed * turnDirection, -speed * turnDirection);
      }*/

      // If we need to turn right
      if(drivetrain.getGyroHeading() < desiredHeading) {
        // While we are to the left of our goal
        while(drivetrain.getGyroHeading() < desiredHeading) {
          // Turn right
          drivetrain.drive(findSpeed, -findSpeed);
        }
      } else { // If we need to turn left
        // While we are to the right of our goal
        while(drivetrain.getGyroHeading() > desiredHeading) {
          // Turn left
          drivetrain.drive(-findSpeed, findSpeed);
        }
      }

      /*if(drivetrain.getGyroHeading() >= desiredHeading) {
        drivetrain.drive(-speed, speed);
        //drivetrain.pivotTurn(-90, true);
      } else {
        drivetrain.drive(speed, -speed);
        //drivetrain.pivotTurn(90, true);
      }*/
    } else {
      powercellX = pixy.getXOffset(); // Get the horizontal alignment to the powercell
      powercellDetected = true;
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
    SmartDashboard.putBoolean("Seeking powercell", false);
    drivetrain.drive(0, 0);
    System.out.println("Aligned with powercell");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished && powercellDetected; // Finish once we've seen a powercell and we're aligned with it
  }
}
