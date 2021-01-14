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

public class LimelightCenter extends CommandBase {

  private final Drivetrain drivetrain;

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry xOffsetEntry = limelight.getEntry("tx");
  NetworkTableEntry yOffset = limelight.getEntry("ty");
  NetworkTableEntry targetArea = limelight.getEntry("ta");

  double minimumSpeed = 0.25;
  double tolerance =  1.5;
  double baseSpeed = 0.30;

  double drivetrainSpeed;

  double xOffset;

  boolean onTarget = false;
  int onTargetCount = 0;
  
  /**
   * Creates a new LimelightCenter.
   */
  public LimelightCenter(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("Minimum Speed", minimumSpeed);
    // SmartDashboard.putNumber("Base Speed", baseSpeed);
    xOffset = xOffsetEntry.getDouble(0.0);
    minimumSpeed =  0.18;
    tolerance = 1.5;
    onTarget = false;
    onTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    xOffset = xOffsetEntry.getDouble(0.0);
    // xOffset will be above 1, which would set our speed to 100% power, so need to divide to make it proportional
    drivetrainSpeed = baseSpeed + ( Math.abs(xOffset) / 100 ); 
    
    //Slow down when close to target
    if(Math.abs(xOffset) < 5) {
      drivetrainSpeed = minimumSpeed + 0.05;
    }

    if(Math.abs( xOffset ) < tolerance ) {
      if(Math.abs( xOffset ) > 0.5) {
        // slow even more when very close, between 1.5 and 0.5
        drivetrainSpeed = minimumSpeed;
        onTargetCount++;
      } else {
        // within 0.5 so stop/coast
        drivetrain.drive(0.0, 0.0);
        onTargetCount++;
      } 
    }

    //Checks if speed is too low
    if(drivetrainSpeed < minimumSpeed){
      drivetrainSpeed = minimumSpeed;
    }
    
    SmartDashboard.putNumber("Speed", drivetrainSpeed);
    SmartDashboard.putNumber("X Offset", xOffset);
    //Checks offsets of center of target and moves robot to center 

    if(xOffset > 0) {
      // Turn right
      drivetrain.drive(drivetrainSpeed, -drivetrainSpeed);
    } else if(xOffset < 0) {
      // Turn left
      drivetrain.drive(-drivetrainSpeed, drivetrainSpeed);
    } 
    
    if(onTargetCount > 25) {
      onTarget = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(xOffset) < 0.5 && (drivetrainSpeed == minimumSpeed);
    return onTarget;
  }
}
