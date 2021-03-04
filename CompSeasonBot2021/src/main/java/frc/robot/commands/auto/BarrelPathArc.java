// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class BarrelPathArc extends SequentialCommandGroup {
  /** Creates a new SlalomPath. */
  Drivetrain drivetrain;


  public BarrelPathArc(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = drive;
    addCommands(new AutoForwardMotionMagic(drivetrain, 120),
                new AutoArcMotionMagic(drivetrain, 33, 360),
                new AutoForwardMotionMagic(drivetrain, 90),
                new AutoArcMotionMagic(drivetrain, 34, -270),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoArcMotionMagic(drivetrain, 31, -90),
                new AutoForwardMotionMagic(drivetrain, 60),
                new AutoArcMotionMagic(drivetrain, 30, -180),
                new AutoForwardMotionMagic(drivetrain, 240)); 

    addCommands();
  }
}
