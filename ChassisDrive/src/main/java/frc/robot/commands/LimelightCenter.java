/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightCenter extends Command {
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry xOffset = limelight.getEntry("tx");
    NetworkTableEntry yOffset = limelight.getEntry("ty");
    NetworkTableEntry targetArea = limelight.getEntry("ta");
  public LimelightCenter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Gets minimum speed from SmartDashboard
    final double minimumSpeed = SmartDashboard.getNumber("Minimum Speed", 0.20);
    //Sets the speed that the robot turns based on area of target on screen
    double drivetrainSpeed = .5; //targetArea.getDouble(0.0)*SmartDashboard.getNumber("Speed to Area Mult", .11);
    //Gets cenetring tolerance from SmartDashboard
    double tolerance = SmartDashboard.getNumber("Tolerance", 2.0);
    //Checks if speed is too low
    if(drivetrainSpeed < minimumSpeed){
      drivetrainSpeed = minimumSpeed;
    }
    if(targetArea.getDouble(0.0) > 1.35){
      drivetrainSpeed = 0.35;
    }
    //Slows the robot when gets close to target
    if(Math.abs(xOffset.getDouble(0.0)) < 7.5){
      drivetrainSpeed = 0.225;
    }
    //Checks offsets of center of target and moves robot to center 
    if(xOffset.getDouble(0.0) > tolerance || xOffset.getDouble(0.0) < -tolerance) {
      SmartDashboard.putNumber("Speed", drivetrainSpeed);
      if(xOffset.getDouble(0.0) > tolerance) {
          // Turn right
          Robot.driveTrain.drive(drivetrainSpeed, -drivetrainSpeed);
      } else if(xOffset.getDouble(0.0) < -tolerance) {
          // Turn left
          Robot.driveTrain.drive(-drivetrainSpeed, drivetrainSpeed);
      } else {
          //Stop Chassis
          Robot.driveTrain.drive(0.0, 0.0);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
