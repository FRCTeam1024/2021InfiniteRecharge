// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BouncePathArc extends SequentialCommandGroup {
  /** Creates a new BouncePathArc. */
  Drivetrain drivetrain;
  public BouncePathArc(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    drivetrain = drive;
    addCommands(
      new AutoForwardMotionMagic(drivetrain, 30),
      new AutoArcMotionMagic(drivetrain, 30, -90),
      new AutoForwardMotionMagic(drivetrain, 10),
      new AutoForwardMotionMagic(drivetrain, -40),
      new AutoArcMotionMagic(drivetrain, -120, -25),
      //new AutoForwardMotionMagic(drivetrain, -45),
      new AutoArcMotionMagic(drivetrain, -30, -150),
      new AutoForwardMotionMagic(drivetrain, -75),
      new AutoForwardMotionMagic(drivetrain, 80),
      new AutoArcMotionMagic(drivetrain, 30, -90),
      new AutoForwardMotionMagic(drivetrain, 35),
      new AutoArcMotionMagic(drivetrain, 30, -90),
      new AutoForwardMotionMagic(drivetrain, 80),
      new AutoForwardMotionMagic(drivetrain, -40),
      new AutoArcMotionMagic(drivetrain,-30, -90)
   
      
      );

      
    
    
  }
}
