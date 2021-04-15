// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixyCam;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AnglePixy extends CommandBase {
  private final PixyCam pixy;

  /** Creates a new AnglePixy. 
   * @param tiltValue integer defining the pixy tilt. -1 denotes current tilt.
   * @param panValue integer defining the pixy pan. -1 denotes current pan.
  */
  public AnglePixy(PixyCam pixy) {
    this.pixy = pixy;
    addRequirements(pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      pixy.setTilt((int) SmartDashboard.getNumber("Servo tilt", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
