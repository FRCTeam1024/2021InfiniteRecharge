// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoTurnGlobal extends CommandBase {
  /** Creates a new AutoTurnMotionMagic. */
  Drivetrain drivetrain;
  double turnAngle;
  int MSstayed = 0; //don't know if this is as necessary for turning, but I'll go ahead and put it in anyways
  public AutoTurnGlobal(Drivetrain drive, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    addRequirements(drivetrain);
    turnAngle = heading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    turnAngle = turnAngle - drivetrain.getGyroHeading();
    System.out.println(drivetrain.getGyroHeading());
    System.out.println("I am turning to "+turnAngle);
    if(turnAngle > 180) {
      turnAngle = 360-turnAngle;
    }
    else if (turnAngle < -180) {
      turnAngle = 360+turnAngle;
    }

  //drivetrain.pivotTurn(angle, false); // low gear turn
    drivetrain.pivotTurn(turnAngle, true); // high gear turn
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end. +/- 1.5 degrees is about the best we can do with encoders because of lash in the gears
  @Override
  public boolean isFinished() {
    if(drivetrain.getHeading() > (turnAngle - 1.5) && drivetrain.getHeading() < (turnAngle + 1.5)){
      MSstayed += 20;
      if(MSstayed > 500){
        return true;
      }else{
        return false;
      }
    }else{
      MSstayed = 0;
      return false;
    }
  }
}
