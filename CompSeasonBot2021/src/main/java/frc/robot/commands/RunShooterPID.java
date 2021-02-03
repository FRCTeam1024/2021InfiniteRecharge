/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterPID extends CommandBase {
  
  private final Shooter m_Shooter;
  private double targetSpeed;
 
  /**
   * Creates a new RunShooterPID.
   * @param targetSpeed The target shooter speed in RPM
   * 	
   */
  public RunShooterPID(Shooter subsystem, double rpm) {
    m_Shooter = subsystem;
    targetSpeed = rpm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    /* Run the Shooter */
    m_Shooter.runControlledShooter(targetSpeed);
    m_Shooter.extendHood();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /* Update target speed from SmartDashboard */
    targetSpeed = SmartDashboard.getNumber("Shooter RPM", 0);
    m_Shooter.runControlledShooter(targetSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
