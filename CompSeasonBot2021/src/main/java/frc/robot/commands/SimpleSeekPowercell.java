// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.DriverAction;
import java.util.Hashtable;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoTurnHeading;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;

public class SimpleSeekPowercell extends CommandBase {
  private final PixyCam pixy;
  private final Drivetrain drivetrain;
  private final int errorThreshold = 2; // Must be within 15 (out of 360) pixels.
  private final double speed = 0.2; // Speed to drive the robot.
  private boolean powercellDetected;
  private double startHeading;
  double powercellX;
  double xError;
  boolean isFinished;

  /** Creates a new SimpleSeekPowercell. */
  public SimpleSeekPowercell(PixyCam pixySubsystem, Drivetrain drivetrainSubsystem) {
    addRequirements(pixySubsystem, drivetrainSubsystem);
    pixy = pixySubsystem;
    drivetrain = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.shiftLow();
    powercellDetected = false;
    isFinished = false;

    startHeading = drivetrain.getGyroHeading();
    SmartDashboard.putBoolean("Seeking powercell", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    powercellX = pixy.getXOffset();
    if(powercellX == -1) { // If we still don't find a powercell
      if(startHeading > 0) {
        drivetrain.drive(-0.5, 0.5);
      } else if(startHeading < 0) {
        drivetrain.drive(0.5, -0.5);
      }
    } else { // If we do see a powercell
      powercellDetected = true;
      xError = powercellX - Constants.PixyConstants.TARGET_X;
      if(Math.abs(xError) <= errorThreshold) {
        isFinished = true;
      } else if(xError > 0) {
        drivetrain.drive(speed, -speed);
      } else if(xError < 0) {
        drivetrain.drive(-speed, speed);
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
    return isFinished && powercellDetected;
  }
}
