/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class FailSafeAutoWithVelocity extends CommandBase {
  /**
   * Creates a new failSafeAuto.
   */
  Shooter shooter;
  BallFeed ballFeed;
  double shooterMotorSpeed;
  double ballFeedSpeed;
  double shooterFeedSpeed;
  public FailSafeAutoWithVelocity(Shooter shooter, BallFeed ballFeed, double shooterMotorSpeed, double ballFeedSpeed, double shooterFeedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.ballFeed = ballFeed;
    this.shooterMotorSpeed = shooterMotorSpeed;
    this.ballFeedSpeed = ballFeedSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isAtMaxRPM() == false){
      shooter.runShooterMotors(shooterMotorSpeed);
    }
    else if(shooter.isAtMaxRPM() == true){
      ballFeed.runBallFeedMotor(ballFeedSpeed);
      ballFeed.runShooterFeedMotor(shooterFeedSpeed);
      shooter.runShooterMotors(shooterMotorSpeed);
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    ballFeed.runBallFeedMotor(0.0);
    ballFeed.runShooterFeedMotor(0.0);
    shooter.runShooterMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
