// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LimelightLED extends CommandBase {
  private final Limelight limelight;
  private final int state;
  private boolean finished;

  /** Creates a new LimelightLED. */
  public LimelightLED(Limelight limelight, int state) {
    this.finished = false;
    this.limelight = limelight;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.toggleLEDs(this.state);
    this.finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
