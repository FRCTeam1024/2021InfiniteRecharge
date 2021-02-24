// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoTurnHeading extends CommandBase {
  /** Creates a new AutoTurnHeading. */
  private final Drivetrain drivetrain;
  private final double heading;
  private double turnAngle;
  private int MSstayed = 0; //don't know if this is as necessary for turning, but I'll go ahead and put it in anyways

  public AutoTurnHeading(Drivetrain drive, double h) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    heading = h;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    while(!drivetrain.gyroReady()) {
      System.out.println("Waiting for the gyro");
    }

    turnAngle = heading - drivetrain.getGyroHeading();

    if(turnAngle > 180) {
      turnAngle = -(360-turnAngle);
    }
    else if (turnAngle < -180) {
      turnAngle = 360+turnAngle;
    }
    
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
