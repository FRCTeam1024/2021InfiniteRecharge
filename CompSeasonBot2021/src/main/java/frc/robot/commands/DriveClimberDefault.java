/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.Logitech;
import frc.robot.oi.MustangController;
import frc.robot.subsystems.Climber;

public class DriveClimberDefault extends CommandBase {

  Logitech controller;
  Climber climber;
  /**
   * Creates a new DriveClimberDefault.
   */
  public DriveClimberDefault(Climber climber, Logitech controller) {
    this.climber = climber;
    this.controller = controller;
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
    SmartDashboard.putNumber("left stick Y", controller.getLeftStickY());
    SmartDashboard.putNumber("right stick Y", controller.getRightStickY());
    
    // invert the right side? it was only running one way
    climber.moveClimber(controller.getLeftStickY(), -controller.getRightStickY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
