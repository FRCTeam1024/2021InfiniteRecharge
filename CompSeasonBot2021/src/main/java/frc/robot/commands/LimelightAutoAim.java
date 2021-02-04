// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAutoAim extends PIDCommand {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  /** Creates a new LimelightAutoAim. */
  public LimelightAutoAim(Limelight limelightInstance, Drivetrain drivetrainInstance) {
    super(
        // The controller that the command will use
        limelightInstance.getPIDController(),
        // This should return the measurement
        () -> limelightInstance.getXOffset(), // Gets the horizontal offset in degrees (which will be used to calculate error)
        // This should return the setpoint (can also be a constant)
        () -> 0.0, // Setpoint is 0.0 degree xOffset, which is centered
        // This uses the output        
        output -> {
          double adjustedOutput = MathUtil.clamp(output, -1.0, 1.0);
          drivetrainInstance.drive(-adjustedOutput, adjustedOutput);
        } //drivetrainInstance.drive(output, -output)
        // Require driveSubsystem?
    );

    // Use addRequirements() here to declare subsystem dependencies.
    limelight = limelightInstance;
    drivetrain = drivetrainInstance;
    addRequirements(limelight);
    // Configure additional PID options by calling `getController` here.
    limelight.getPIDController().setTolerance(1); // 1 degree for now // limelight.getPIDController().enableContinuousInput(-29.8, 29.8); // min and max degrees on the limelight
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getPIDController().atSetpoint();
  }
}
