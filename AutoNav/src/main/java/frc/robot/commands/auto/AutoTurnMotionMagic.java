// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoTurnMotionMagic extends CommandBase {
  /** Creates a new AutoTurnMotionMagic. */
  Drivetrain drivetrain;
  double angle;
  int MSstayed = 0; //don't know if this is as necessary for turning, but I'll go ahead and put it in anyways
  public AutoTurnMotionMagic(Drivetrain drive, double turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    addRequirements(drivetrain);
    angle = turn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drivetrain.pivotTurn(angle, false); // low gear turn
    drivetrain.pivotTurn(angle, true); // high gear turn
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
    if(drivetrain.getHeading() > (angle - 0.9) && drivetrain.getHeading() < (angle + .9)){
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
