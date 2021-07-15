/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShootPowerCell extends ParallelCommandGroup {
  /**
   * Creates a new ShootPowerCell.
   * 
   * I don't think this will work right. I think we can just use runballfeedandshooterfeed instead.  Intak
   * will already be running via a seperate command.
   */
  

  public ShootPowerCell(Intake intake, BallFeed ballFeed, Drivetrain drivetrain, Shooter shooter) {
    super(new RunIntake(intake, 1.0), new RunShooterFeed(ballFeed, 1.0), new RunBallFeed(ballFeed, -0.50));
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    
  }
}
