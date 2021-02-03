// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoForwardMotionMagic extends CommandBase {
  /** Creates a new AutoForwardMotionMagic. */
  Drivetrain drivetrain;
  double dist;
  double distTraveled;
  public AutoForwardMotionMagic(Drivetrain drive, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    addRequirements(drivetrain);
    dist = inches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.driveStraight(dist, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getDistance() > (dist * 0.95) && drivetrain.getDistance() < (dist * 1.05)){
      return true;
    }else{
      return false;
    }
  }
}
