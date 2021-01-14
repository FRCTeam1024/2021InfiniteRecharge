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
  private int sequences = 0;
  public double drivetrainSpeed = .4;

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry xOffset = limelight.getEntry("tx");
  NetworkTableEntry yOffset = limelight.getEntry("ty");
  NetworkTableEntry targetArea = limelight.getEntry("ta");
  
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
    sequences = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double minimumSpeed = SmartDashboard.getNumber("Minimum Speed", 0.20);
    
    drivetrainSpeed = .4; 
    //Gets centering tolerance from SmartDashboard
    double tolerance = SmartDashboard.getNumber("Tolerance", 1.5);
    
    //Slows the robot when gets close to target
    if(Math.abs(xOffset.getDouble(0.0)) < 7.5){
      drivetrainSpeed = 0.225;
    }

    if(Math.abs( xOffset.getDouble(0.0) ) < tolerance ) {
      if(Math.abs( xOffset.getDouble(0.0) ) > 0.5) {
        // slow even more when very close, between 1.5 and 0.5
        drivetrainSpeed = minimumSpeed;
      } else {
        // within 0.5 so stop/coast
        drivetrain.drive(0.0, 0.0);
        sequences++;
      } 
    }

    //Checks if speed is too low
    if(drivetrainSpeed < minimumSpeed){
      drivetrainSpeed = minimumSpeed;
    }
    
    SmartDashboard.putNumber("Speed", drivetrainSpeed);
    SmartDashboard.putNumber("X Offset", xOffset.getDouble(0.0));
    //Checks offsets of center of target and moves robot to center 

    if(xOffset.getDouble(0.0) > tolerance) {
        // Turn right
      drivetrain.drive(drivetrainSpeed, -drivetrainSpeed);
    } else if(xOffset.getDouble(0.0) < -tolerance) {
        // Turn left
        drivetrain.drive(-drivetrainSpeed, drivetrainSpeed);
    } 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sequences >= 15 || drivetrainSpeed == 0.0;
  }
}
