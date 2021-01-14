/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDDrivetrain;

public class TurnToAnglePID extends PIDCommand {

  PIDDrivetrain drivetrain;
  /**
   * Creates a new TurnToAnglePID.
   */
  public TurnToAnglePID(double targetAngleDegrees, PIDDrivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(PIDDrivetrain.Kp, PIDDrivetrain.Ki, PIDDrivetrain.Kd),
        // This should return the measurement
        drivetrain::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> {
          System.out.println("output : " + output);
          double power = output / 180;
          double minPower = 0.25;
          if(power < minPower) {
            power = minPower; 
          }
          // Use the output here
          drivetrain.drive(power, -power);
        },
        // require the drivetrain
        drivetrain);
        
    this.drivetrain = drivetrain;

    // Configure additional PID options by calling `getController` here.
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(3, 45);
  
  }

  @Override
  public void initialize() {
    drivetrain.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
