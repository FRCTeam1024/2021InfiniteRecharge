// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class AutoPIDShoot extends CommandBase {
  private final Hood hood;
  private final Limelight limelight;
  private final BallFeed ballfeed;
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private boolean finished;

  /** Creates a new AutoPIDShoot. */
  public AutoPIDShoot(Hood hood, Limelight limelight, BallFeed ballfeed, Drivetrain drivetrain, Shooter shooter) {
    this.hood = hood;
    this.limelight = limelight;
    this.ballfeed = ballfeed;
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood, limelight, ballfeed, drivetrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new ExtendHood(this.hood);
    this.limelight.setLEDState(1);
    new RunBothFeeders(this.ballfeed);
    new RunShooterPID(this.shooter, 2500);
    new WaitCommand(2.0);
    /*if(this.limelight.hasTarget()) {
      if(this.limelight.getXOffset() < this.limelight.getThreshold() && this.limelight.getXOffset() > -this.limelight.getThreshold()) {
        new LimelightAutoAim(this.limelight, this.drivetrain);
      }
      new RunBothFeeders(this.ballfeed);
      new RunShooterPID(this.shooter, 2500);
      new WaitCommand(2.0);
    }*/

    this.finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.limelight.setLEDState(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}

