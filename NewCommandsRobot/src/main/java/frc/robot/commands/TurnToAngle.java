/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;

public class TurnToAngle extends CommandBase {
  Drivetrain drivetrain;
  Sensors sensors;
  double startHeading;
  int targetAngle;
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(int targetAngle, Drivetrain drivetrain, Sensors sensors) {
    this.drivetrain = drivetrain;
    this.sensors = sensors;
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.resetGyro();
    startHeading = sensors.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.turnRight(40);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 5 is our tolerance; gives some time to coast into target and not overshoot
    return Math.abs(sensors.getHeading() - targetAngle) < 5;
  }
}
