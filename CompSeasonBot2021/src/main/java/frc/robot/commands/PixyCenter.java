// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;

public class PixyCenter extends CommandBase {
  private final PixyCam m_Pixy;
  private final Drivetrain m_Drivetrain;
  private double powerCellX;
  private int sequences = 0;
  public double driveTrainSpeed = .4;
  private final double minimumSpeed = .2;
  private double tolerance = 1.5;

  /** Creates a new PixyCenter. */
  public PixyCenter(PixyCam p, Drivetrain drive) {
    m_Pixy = p;
    m_Drivetrain = drive;
    addRequirements(m_Pixy, m_Drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sequences = 0;
    powerCellX = m_Pixy.getXOffset();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powerCellX = m_Pixy.getXOffset();

    if(Math.abs(powerCellX) < 7.5){
      driveTrainSpeed = 0.225;
    }

    if(Math.abs(powerCellX) < tolerance ) {
      if(Math.abs(powerCellX) > 0.5) {
        // slow even more when very close, between 1.5 and 0.5
        driveTrainSpeed = minimumSpeed;
      } else {
        // within 0.5 so stop/coast
        m_Drivetrain.drive(0.0, 0.0);
        sequences++;
      } 
    }

    if(driveTrainSpeed < minimumSpeed){
      driveTrainSpeed = minimumSpeed;
    }

    if(this.powerCellX > tolerance) {
      // Turn right
    m_Drivetrain.drive(driveTrainSpeed, -driveTrainSpeed);
  } else if(this.powerCellX < -tolerance) {
      // Turn left
      m_Drivetrain.drive(-driveTrainSpeed, driveTrainSpeed);
  } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return sequences >= 1 || driveTrainSpeed <= 0.1;
    return false;
  }
}