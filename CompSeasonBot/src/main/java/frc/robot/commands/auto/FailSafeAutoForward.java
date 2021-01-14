/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FailSafeAutoForward extends CommandBase {
  /**
   * Creates a new FailSafeAuto.
   */
  Shooter shooter;
  BallFeed ballFeed;
  double shooterSpeed;
  double ballFeedSpeed;
  double shooterFeedSpeed;
  Drivetrain drivetrain;
  Boolean isFinished;
  public FailSafeAutoForward(Drivetrain drivetrain, Shooter shooter, BallFeed ballFeed, double shooterSpeed, double ballFeedSpeed, double shooterFeedSpeed) {
    this.shooter = shooter;
    this.ballFeed = ballFeed;
    this.drivetrain = drivetrain;
    this.shooterSpeed = shooterSpeed;
    this.ballFeedSpeed = ballFeedSpeed;
    this.shooterFeedSpeed = shooterFeedSpeed;
    isFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runShooterMotors(shooterSpeed);
    Timer.delay(3);
   // Timer.delay(3);
    ballFeed.runShooterFeedMotor(shooterFeedSpeed);
    ballFeed.runBallFeedMotor(ballFeedSpeed);
    
    
    Timer.delay(0.15);

    ballFeed.stopBallFeedMotor();
    ballFeed.stopShooterFeedMotor();
    Timer.delay(3);

    ballFeed.runShooterFeedMotor(shooterFeedSpeed);
    ballFeed.runBallFeedMotor(ballFeedSpeed);
    Timer.delay(0.15);

    ballFeed.stopBallFeedMotor();
    ballFeed.stopShooterFeedMotor();
    Timer.delay(3);

    ballFeed.runShooterFeedMotor(shooterFeedSpeed);
    ballFeed.runBallFeedMotor(ballFeedSpeed);

    Timer.delay(0.15);
    ballFeed.stopBallFeedMotor();
    ballFeed.stopShooterFeedMotor();

    isFinished = true;

    drivetrain.drive(-.5, -.5);
    Timer.delay(1);
    drivetrain.drive(0.0, 0.0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    shooter.runShooterMotors(0.0);
    ballFeed.runShooterFeedMotor(0.0);
    ballFeed.runBallFeedMotor(0.0);
    drivetrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
