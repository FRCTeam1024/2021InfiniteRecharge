// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithIntake;
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
  /** Creates a new GalacticSearch. */
  public GalacticSearch(Drivetrain drivetrain, Intake intake, PixyCam pixy, BallFeed ballfeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Calibrate(drivetrain),
                new RetractIntake(intake),
                new SimpleSeekPowercell(pixy, drivetrain),
                new DriveWithIntake(intake, ballfeed, drivetrain, pixy),
                new DriveWithIntake(intake, ballfeed, drivetrain, pixy),
                new AutoTurnMotionMagic(drivetrain, -90),
                new DriveWithIntake(intake, ballfeed, drivetrain, pixy),
                new AutoTurnHeading(drivetrain, 0),
                new AutoForwardMotionMagic(drivetrain, 1000)    
    );
  }
}
