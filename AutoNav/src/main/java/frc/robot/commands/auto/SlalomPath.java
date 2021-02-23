// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class SlalomPath extends SequentialCommandGroup {
  /** Creates a new SlalomPath. */
  Drivetrain drivetrain;


  public SlalomPath(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = drive;
    addCommands(new AutoForwardMotionMagic(drivetrain, 45), 
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 85), 
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 180), 
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 75), 
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 75),
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 70),
                new AutoTurnMotionMagic(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 170),
                new AutoTurnMotionMagic(drivetrain, 90), 
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnMotionMagic(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 40)); 

/* Test using global headings from gyro, not working yet
    addCommands(new AutoForwardMotionMagic(drivetrain, 45), 
                new AutoTurnGlobal(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 85), 
                new AutoTurnGlobal(drivetrain, 0),
                new AutoForwardMotionMagic(drivetrain, 180), 
                new AutoTurnGlobal(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 75), 
                new AutoTurnGlobal(drivetrain, 0),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnGlobal(drivetrain, -90),
                new AutoForwardMotionMagic(drivetrain, 75),
                new AutoTurnGlobal(drivetrain, -180),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnGlobal(drivetrain, 90),
                new AutoForwardMotionMagic(drivetrain, 70),
                new AutoTurnGlobal(drivetrain, -180),
                new AutoForwardMotionMagic(drivetrain, 170),
                new AutoTurnGlobal(drivetrain, -90), 
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoTurnGlobal(drivetrain, -180),
                new AutoForwardMotionMagic(drivetrain, 40));
                */

    addCommands();
  }
}
