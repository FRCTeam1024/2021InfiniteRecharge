/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimber extends CommandBase {
  /**
   * Creates a new RunClimber.
   */
  Climber climber;
  double motorSpeedOne;
  double motorSpeedTwo;
  public RunClimber(Climber climber, double motorSpeedOne, double motorSpeedTwo) {
    this.climber = climber;
    this.motorSpeedOne = motorSpeedOne;    
    this.motorSpeedTwo = motorSpeedTwo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.moveClimber(motorSpeedOne, motorSpeedTwo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climberMotorLeft.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
