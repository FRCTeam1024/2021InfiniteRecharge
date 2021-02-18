// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyCam;
import edu.wpi.first.wpiutil.math.MathUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimPixyPowercell extends PIDCommand {
  private final PixyCam pixy;
  private final Drivetrain driveTrain;

  /** Creates a new aimPixyPowercell. */
  public AimPixyPowercell(PixyCam pixy, Drivetrain driveTrain) {
    super(
        // The controller that the command will use
        pixy.getPIDController(),
        //new PIDController(0, 0, 0),
        // This should return the measurement
        () -> pixy.getXOffset(),
        // This should return the setpoint (can also be a constant)
        () -> 180.0,
        // This uses the output
        output -> {
          // Use the output here
          double adjustedOutput = MathUtil.clamp(output, -1.0, 1.0);
          driveTrain.drive(adjustedOutput, -adjustedOutput);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.pixy = pixy;
    this.driveTrain = driveTrain;
    addRequirements(pixy, driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.pixy.getPIDController().atSetpoint();
  }
}
