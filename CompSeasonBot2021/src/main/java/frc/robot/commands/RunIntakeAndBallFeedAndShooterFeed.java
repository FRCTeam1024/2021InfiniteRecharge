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

public class RunIntakeAndBallFeedAndShooterFeed extends CommandBase {
  /**
   * Creates a new RunIntake.
   */
  Intake intake;
  BallFeed ballFeed;
    double intakeSpeed;
    double ballFeedSpeed;
    double shooterFeedSpeed;
  public RunIntakeAndBallFeedAndShooterFeed(Intake intake, BallFeed ballFeed, double intakeSpeed, double ballFeedSpeed, double shooterFeedSpeed) {
    this.intake = intake;
    this.ballFeed = ballFeed;
    this.intakeSpeed = intakeSpeed;
    this.ballFeedSpeed = ballFeedSpeed;
    this.shooterFeedSpeed = shooterFeedSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(intakeSpeed);
    ballFeed.runBallFeedMotor(ballFeedSpeed);
    ballFeed.runShooterFeedMotor(shooterFeedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    ballFeed.stopBallFeedMotor();
    ballFeed.stopShooterFeedMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
