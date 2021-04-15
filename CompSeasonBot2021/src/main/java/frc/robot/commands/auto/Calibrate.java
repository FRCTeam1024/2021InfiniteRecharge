// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Calibrate extends CommandBase {
  /** Creates a new Calibrate. */
  private final Drivetrain drivetrain;

  public Calibrate(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stop(); // Make sure we aren't moving
    drivetrain.calibrateGyro(); //Run calibration on gyro
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
  }

  // Returns true when the command should end. +/- 1.5 degrees is about the best we can do with encoders because of lash in the gears
  @Override
  public boolean isFinished() {
    return drivetrain.gyroReady();
  }
}
