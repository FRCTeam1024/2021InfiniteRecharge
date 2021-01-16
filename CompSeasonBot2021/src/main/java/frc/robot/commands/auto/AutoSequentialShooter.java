/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Shooter;

// make it a RaceGroup so it will end when the SequentialShooter ends, 
// which it should do after shooting 3 balls
// the RunShooter does not currently have an end condition
public class AutoSequentialShooter extends ParallelRaceGroup {
  /**
   * Creates a new AutoSequentialShooter.
   */
  public AutoSequentialShooter(Shooter shooter, BallFeed ballFeed) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new RunShooter(shooter, 1.0), new SequentialShooter(shooter, ballFeed));
  }
}