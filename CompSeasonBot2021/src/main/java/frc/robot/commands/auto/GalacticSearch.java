// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CollectNearestPowercell;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.SimpleSeekPowercell;
import frc.robot.subsystems.BallFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PixyCam;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  /**
   * This command group will run Path A for galactic search.
   * @param drivetrain
   * @param intake
   * @param pixy
   * @param ballfeed
   */
  public GalacticSearch(Drivetrain drivetrain, Intake intake, PixyCam pixy, BallFeed ballfeed) {
    /**
     * @TODO: Find a way to combine SimpleSeekPowercell and CollectNearestPowercell into one command
     * See if it's possible to call SimpleSeekPowercell within CollectNearestPowercell?
     */
    addCommands(new Calibrate(drivetrain), // Reset heading to 0 when we're straight on
                new RetractIntake(intake), // Lower the intake
                new SimpleSeekPowercell(pixy, drivetrain), // Align to the nearest powercell
                new CollectNearestPowercell(intake, ballfeed, drivetrain, pixy), // Collect it
                // Repeat 2x
                new SimpleSeekPowercell(pixy, drivetrain), 
                new CollectNearestPowercell(intake, ballfeed, drivetrain, pixy),
                new SimpleSeekPowercell(pixy, drivetrain),
                new CollectNearestPowercell(intake, ballfeed, drivetrain, pixy),
                new AutoTurnHeading(drivetrain, 0), // Align towards the end zone
                new AutoForwardMotionMagic(drivetrain, 1000) // Drive forwards
    );
  }
}