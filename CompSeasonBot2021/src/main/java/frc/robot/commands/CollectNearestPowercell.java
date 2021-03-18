// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PixyCam;

/**
 * This command will drive forward until the intake collects a powercell
 * @TODO: Align the robot with the powercell as it drives? Implement SimpleSeekPowercell for this
 */
public class CollectNearestPowercell extends CommandBase {
  private final Intake intake;
  private final BallFeed ballfeed;
  private final Drivetrain drivetrain;
  private final PixyCam pixy;

  private double speed = 7; // feet per second

  private boolean hasPowercell;

  /**
   * This command collects a powercell directly in front of the robot.
   * @param intakeSubsystem
   * @param ballfeedSubsystem
   * @param drivetrainSubsystem
   * @param pixySubsystem
   */
  public CollectNearestPowercell(Intake intakeSubsystem, BallFeed ballfeedSubsystem, Drivetrain drivetrainSubsystem, PixyCam pixySubsystem) {
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
    drivetrain.shiftLow(); // Shift into low gear

    System.out.println("Going to get powercell");

    intake.runIntake(0.35); // Run the intake
    ballfeed.runBallFeedMotor(0.75); // Run the ballfeed motor
    drivetrain.driveSpeed(speed, false); // Drive forward at 5ft/sec in low gear 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pixy.getYOffset() > 185 || pixy.getYOffset() == -1) { // If the powercell is within the relative area of our intake
      Timer.delay(1); // Wait 1 second for it to be collected
      hasPowercell = true; // The robot has the powercell, we're finished
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
    // If this command is ended or interrupted, stop all motors.
    intake.runIntake(0);
    ballfeed.runBallFeedMotor(0);
    drivetrain.drive(0, 0);
    System.out.println("I got the powercell!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPowercell; // If we have a powercell, we're done.
  }
}
