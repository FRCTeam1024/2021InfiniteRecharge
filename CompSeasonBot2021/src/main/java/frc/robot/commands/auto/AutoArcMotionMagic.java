// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoArcMotionMagic extends CommandBase {
  /** Creates a new AutoArcMotionMagic. */
  Drivetrain drivetrain;
  double radius;
  double angle;
  int MSstayed = 0;
  public AutoArcMotionMagic(Drivetrain drive, double theRadius, double theAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    angle = theAngle;
    radius = theRadius;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.driveArc(radius, angle, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getHeading() > (angle - 1) && drivetrain.getHeading() < (angle + 1)){
      if(drivetrain.getDistance() > (2*3.14159*radius*(Math.abs(angle)/360) - .5) && drivetrain.getDistance() < (2*3.14159*radius*(Math.abs(angle)/360) + .5)){
        MSstayed+=20;
        if(MSstayed > 250){
          return true;
        }else{
          return false;
        }
      }else{
        MSstayed = 0;
        return false;
      }
    }else{
      MSstayed = 0;
      return false;
    }
  }
}
