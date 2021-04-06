// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.AutoForwardMotionMagic;
import frc.robot.commands.auto.AutoTurnHeading;
import frc.robot.subsystems.Drivetrain;

// This command will align itself with the finish zone and drive towards it
public class GalacticSearchFinishLine extends CommandBase {
  private final Drivetrain drivetrain;

  private boolean isFinished;

  /** Creates a new GalacticSearchFinishLine. */
  public GalacticSearchFinishLine(Drivetrain drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drivetrainSubsystem;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // @TODO: Test this tomorrow; it may not run the command
    if(Math.abs(drivetrain.getGyroHeading()) > 90) { // If we are more than 90 degrees from the finish zone
      new AutoTurnHeading(drivetrain, 0); // Turn towards the heading of 0
    }

    drivetrain.drive(0.5, 0.5); // Drive towards the finish zone

    // The command doesn't need to end; driver must stop it.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
