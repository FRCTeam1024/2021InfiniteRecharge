// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RectanglePath extends SequentialCommandGroup {
  /** Creates a new RectanglePath. */
  Drivetrain drivetrain;
  public RectanglePath(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = drive;
    //Increased the size of the box from 30 to 120 to give more visibility to turn errors
    /*addCommands(new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnMotionMagic(drivetrain, 90)); */

    addCommands(new Calibrate(drivetrain),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnHeading(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnHeading(drivetrain, 180),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnHeading(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 120), 
                new AutoTurnHeading(drivetrain, 0));
  }
}
