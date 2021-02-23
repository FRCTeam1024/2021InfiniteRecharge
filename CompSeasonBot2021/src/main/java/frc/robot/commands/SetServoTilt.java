// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetServoTilt extends CommandBase {
  /** Creates a new SetServoTilt. */
  final PixyCam pixy;
  int tilt;
  Boolean isFinished;

  public SetServoTilt(PixyCam subsystem, int position) {
    this.pixy = subsystem;
    this.tilt = position;
    this.isFinished = false;
    //addRequirements(pixy);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.pixy.setTilt(tilt);
    this.isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}