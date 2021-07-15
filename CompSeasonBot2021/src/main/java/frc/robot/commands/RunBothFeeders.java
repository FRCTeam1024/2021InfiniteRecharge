/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunBothFeeders extends CommandBase {
  /**
   * Creates a new RunBothFeeders.
   */
  BallFeed ballFeed;
    double ballFeedSpeed;
    double shooterFeedSpeed;
  public RunBothFeeders(BallFeed ballFeed, double ballFeedSpeed, double shooterFeedSpeed) {
    this.ballFeed = ballFeed;
    this.ballFeedSpeed = ballFeedSpeed;
    this.shooterFeedSpeed = shooterFeedSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballFeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballFeed.runBallFeedMotor(ballFeedSpeed);
    ballFeed.runShooterFeedMotor(shooterFeedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballFeed.stopBallFeedMotor();
    ballFeed.stopShooterFeedMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
