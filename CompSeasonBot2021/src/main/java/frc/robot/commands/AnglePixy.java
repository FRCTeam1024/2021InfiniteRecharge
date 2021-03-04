// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixyCam;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AnglePixy extends CommandBase {
  private final PixyCam pixy;
  private Joystick leftJoystick;
  boolean isFinished;

  /** Creates a new AnglePixy. 
   * @param tiltValue integer defining the pixy tilt. -1 denotes current tilt.
   * @param panValue integer defining the pixy pan. -1 denotes current pan.
  */
  public AnglePixy(PixyCam pixy, Joystick leftJoystick) {
    this.isFinished = false;
    this.pixy = pixy;
    this.leftJoystick = leftJoystick;

    addRequirements(pixy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (leftJoystick.getRawButtonPressed(2))
    {
      pixy.setTilt((int) SmartDashboard.getNumber("Servo tilt", 0));
    }
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
/**
public class AnglePixy extends CommandBase {
  private final PixyCam pixy;
  private int tilt, pan;
  boolean isFinished;

   Creates a new AnglePixy. 
    @param tiltValue integer defining the pixy tilt. -1 denotes current tilt.
    @param panValue integer defining the pixy pan. -1 denotes current pan.
  
  public AnglePixy(PixyCam pixySubsystem, int tiltValue, int panValue) {
    this.isFinished = false;
    this.pixy = pixySubsystem;

    this.tilt = (tiltValue != -1) ? tiltValue : this.pixy.getTilt();
    this.pan = (panValue != -1) ? panValue : this.pixy.getPan();

    addRequirements(pixySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.pixy.setTilt(this.tilt);
    this.pixy.setPan(this.pan);
    this.isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
*/