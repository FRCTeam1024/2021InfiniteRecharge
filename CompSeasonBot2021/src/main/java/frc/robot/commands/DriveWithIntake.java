// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.AutoForwardMotionMagic;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class DriveWithIntake extends CommandBase {
  private final Intake intake;
  private final BallFeed ballfeed;
  private final Drivetrain drivetrain;
  private final PixyCam pixy;
  private Block targettedBlock;

  boolean hasPowercell;

  /** Creates a new DriveWithIntake. */
  public DriveWithIntake(Intake intakeSubsystem, BallFeed ballfeedSubsystem, Drivetrain drivetrainSubsystem, PixyCam pixySubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSubsystem;
    ballfeed = ballfeedSubsystem;
    drivetrain = drivetrainSubsystem;
    pixy = pixySubsystem;
    addRequirements(intake, ballfeed, drivetrain, pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasPowercell = false;
    // Gets the index of the very last block in the 
    targettedBlock = pixy.largestBlocks.get(pixy.largestBlocks.size() - 1);
    drivetrain.shiftLow();

    intake.runIntake(0.35);
    ballfeed.runBallFeedMotor(0.75);
    drivetrain.driveSpeed(5, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pixy.getYOffset() > 185 || pixy.getYOffset() == -1) {
      Timer.delay(1);
      System.out.println("Powercell achieved.");
      hasPowercell = true;
    }
    // At the very beggining,  this command stores 
    /* if(pixy.getLargestBlock() != targettedBlock)
    {
      Timer.delay(1);
      System.out.println("Powercell achieved.");
      hasPowercell = true;
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    ballfeed.runBallFeedMotor(0);
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPowercell;
  }
}
