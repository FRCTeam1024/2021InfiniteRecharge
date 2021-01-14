
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;  

public class PIDDriveTrainAim extends PIDCommand {

  public PIDDriveTrainAim( PIDDriveTrain drive) {
    super(
        // The controller that the command will use
        new PIDController(1.0, 0, 0),
        // This should return the measurement
        () -> Robot.getLimelightXOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> drive.useOutput(output, 0 ),
        // this requires the drive
        drive 
        );

    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}