/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAnglePID extends PIDCommand {
  /**
   * Creates a new TurnToAnglePID.
   */
  public TurnToAnglePID(double targetAngleDegrees, Drivetrain drive, Sensors sensors) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // This should return the measurement
        sensors::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> {
          System.out.println("TurnToAnglePID, PIDController output : " + output);
          // Use the output here
          // output comes in the range of the error, so it can be 180 to -180
          // we need to convert that proportionally to our usable numbers, into a %, from 0.0 to 1.0
          double percentPower = Math.abs(output) / targetAngleDegrees;
          System.out.println("TurnToAnglePID, percent power : " + percentPower);
          drive.pivotTurnRight(percentPower, percentPower);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
