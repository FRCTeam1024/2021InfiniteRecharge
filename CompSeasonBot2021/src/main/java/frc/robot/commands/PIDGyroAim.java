/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.*;
import java.lang.Math;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDGyroAim extends CommandBase {

  private Drivetrain drivetrain;
  private int P, I, D = 1;
  private int integral = 0;
  private double setpoint = 0.0;
  private double previous_error = 0.0;
  /**
   * Creates a new PIDGyroAim.
   */
  public PIDGyroAim(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.ahrs.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = setpoint - drivetrain.ahrs.getAngle();
    this.integral += (error*.02);
    double derivative = (error - this.previous_error) /.02;
    double rcw = P*error + I*this.integral + D*derivative;
    this.previous_error = error;
    drivetrain.drive(-rcw, rcw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(drivetrain.ahrs.getAngle() - setpoint) < .5) {
      return true;
    } else {
      return false;
    }
  }
}
