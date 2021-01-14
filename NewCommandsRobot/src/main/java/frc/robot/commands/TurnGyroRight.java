/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TurnGyroRight extends CommandBase {
  /**
   * Creates a new TurnGyroRight.
   * 
   */
  Drivetrain drivetrain;
  double leftPower;
  double rightPower;
  double targetAngle;
  boolean isFinished;
  double slowDownTolerance;

  public TurnGyroRight(Drivetrain drivetrain, double leftPower, double rightPower, double targetAngle) {
    this.drivetrain = drivetrain;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    this.targetAngle = targetAngle;
    slowDownTolerance = 10;
    isFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.ahrs.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.ahrs.getYaw() < targetAngle){
      drivetrain.pivotTurnRight(leftPower, rightPower);
      isFinished = false;
    } else if (drivetrain.ahrs.getYaw() > targetAngle - slowDownTolerance){
      drivetrain.pivotTurnRight(0.2, 0.2);
    }
    else if(drivetrain.ahrs.getYaw() >= targetAngle){
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
