// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;  //DLP: Don't setup a PID controller here, that is already done in the subsystem
import edu.wpi.first.wpilibj2.command.PIDCommand;  //DLP: This should not be a PIDCommand, should extend CommandBase
import java.util.function.*;

/* DLP:
 * This command should extend commandBase and there should be no PIDcontroller objects
 * created here. The PID is handled in the subsystem class.
 * 
 * This command should accept a distance parameter in inches and pass it 
 * to the driveStraight method of the Drivetrain object. (use low gear for now)
 * 
 * This command should then periodically check the distance traveled via the getDistance() method
 * of the Drivetrain object and compare this to the target distance parameter.
 * 
 * The isFinished() method should return true when the distance traveled is within 
 * some predetermined threshold of target distance and it has remained there for
 * a predetermined period of time.
 * 
 * This command should override the End method and command the Drivetrain object to stop.
 */


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardMotionMagic extends PIDCommand {
  /** Creates a new DriveForwardMotionMagic. */
  Drivetrain drivetrain;
  public DriveForwardMotionMagic(Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(Constants.lowGearkP, Constants.lowGearkI, Constants.lowGearkD),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output .
          
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
