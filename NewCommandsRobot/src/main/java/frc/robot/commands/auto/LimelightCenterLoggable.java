/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LimelightCenterLoggable extends CommandBase implements Loggable {

  private final Drivetrain drivetrain;

  @Log
  double minimumSpeed = 0.25;
  @Log
  double tolerance =  1.5;
  @Log
  double baseSpeed = 0.50;

  @Log
  double drivetrainSpeed;

  @Log
  double xOffset;

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry xOffsetEntry = limelight.getEntry("tx");
  NetworkTableEntry yOffset = limelight.getEntry("ty");
  NetworkTableEntry targetArea = limelight.getEntry("ta");
  
  /**
   * Creates a new LimelightCenter.
   */
  public LimelightCenterLoggable(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  @Config
  public void setBaseSpeed(double baseSpeed) {
    this.baseSpeed = baseSpeed;
  }

  @Config
  public void setMinimumSpeed(double minimumSpeed) {
    this.minimumSpeed = minimumSpeed;
  }

  @Config
  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xOffset = xOffsetEntry.getDouble(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    xOffset = xOffsetEntry.getDouble(0.0);
    // xOffset will be above 1, which would set our speed to 100% power, so need to divide to make it proportional
    drivetrainSpeed = baseSpeed + ( Math.abs(xOffset) / 100 ); 
    
    //Slows the robot when gets close to target
    if(Math.abs(xOffset) < 7.5) {
      drivetrainSpeed = 0.225 + (xOffset / 100);
    }

    if(Math.abs( xOffset ) < tolerance ) {
      if(Math.abs( xOffset ) > 0.75) {
        // slow even more when very close, between 1.5 and 0.5
        drivetrainSpeed = minimumSpeed;
      } else {
        // within 0.75 so stop/coast
        drivetrain.drive(0.0, 0.0);
      } 
    }

    //Checks if speed is too low
    if(drivetrainSpeed < minimumSpeed){
      drivetrainSpeed = minimumSpeed;
    }
    
    SmartDashboard.putNumber("Speed", drivetrainSpeed);
    SmartDashboard.putNumber("X Offset", xOffset);
    //Checks offsets of center of target and moves robot to center 

    if(xOffset > tolerance) {
        // Turn right
      drivetrain.drive(drivetrainSpeed, -drivetrainSpeed);
    } else if(xOffset < -tolerance) {
        // Turn left
        drivetrain.drive(-drivetrainSpeed, drivetrainSpeed);
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.setCoastMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs( xOffset) < 0.5;
  }
}
