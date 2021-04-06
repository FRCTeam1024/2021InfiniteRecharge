// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ResetOdometry extends CommandBase {
  /** Creates a new ResetOdometry. */
  Drivetrain drive;
  int path;
  public ResetOdometry(Drivetrain drivetrain, int path) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = drivetrain;
    this.path = path;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(path == 2){
      drive.resetOdometry(new Pose2d(1.95, 3.81, new Rotation2d(Constants.PI/2)));
    }else if(path == 3){
      drive.resetOdometry(new Pose2d(4.572, 3.81, new Rotation2d(-1 * Constants.PI/2)));
    }else if(path == 4){
      drive.resetOdometry(new Pose2d(6.858, 3.81, new Rotation2d(Constants.PI/2)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
