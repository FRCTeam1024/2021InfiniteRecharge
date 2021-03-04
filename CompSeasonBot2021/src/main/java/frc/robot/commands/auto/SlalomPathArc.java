// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class SlalomPathArc extends SequentialCommandGroup {
  /** Creates a new SlalomPath. */
  Drivetrain drivetrain;


  public SlalomPathArc(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = drive;
    addCommands(new AutoForwardMotionMagic(drivetrain, 25),
                new AutoArcMotionMagic(drivetrain, 30, -90),
                new AutoArcMotionMagic(drivetrain, 30, 90),
                new AutoForwardMotionMagic(drivetrain, 120),
                new AutoArcMotionMagic(drivetrain, 30, 90),
                new AutoArcMotionMagic(drivetrain, 30, -360),
                new AutoArcMotionMagic(drivetrain, 30, 90),
                new AutoForwardMotionMagic(drivetrain, 120),
                new AutoArcMotionMagic(drivetrain, 30, 90),
                new AutoArcMotionMagic(drivetrain, 30, -90)); 

    addCommands();
  }
}
