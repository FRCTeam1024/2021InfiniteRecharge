// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;

public class SimpleSeekPowercell extends CommandBase {
  private final PixyCam pixy;
  private final Drivetrain drivetrain;
  private final int errorThreshold = 15; // Must be within 15 (out of 360) pixels.
  private final double speed = 0.5; // Speed to drive the robot.
  double powercellX;
  double xError;
  boolean isFinished = false;

  /** Creates a new SimpleSeekPowercell. */
  public SimpleSeekPowercell(PixyCam pixySubsystem, Drivetrain drivetrainSubsystem) {
    addRequirements(pixySubsystem, drivetrainSubsystem);
    pixy = pixySubsystem;
    drivetrain = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powercellX = pixy.getXOffset();
    if(powerCellX == -1) {
      isFinished = true; // End command if no object is detected for now...
    }
    
    xError = powercellX - 180;
    if(Math.abs(xError) <= errorThreshold) {
      isFinished = true;
    } else if(xError < 180) {
      drivetrain.drive(speed, -speed);
    } else if(xError > 180) {
      drivetrain.drive(-speed, speed);
    } else {
      drivetrain.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
