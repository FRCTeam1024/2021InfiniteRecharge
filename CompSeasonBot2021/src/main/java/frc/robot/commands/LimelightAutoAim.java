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
          double adjustedOutput = MathUtil.clamp(output, limelightInstance.getMinOutput(), limelightInstance.getMinOutput());
          SmartDashboard.putNumber("xOffset", limelightInstance.getXOffset());
          SmartDashboard.putNumber("Output", output);
          SmartDashboard.putNumber("Final Output", adjustedOutput);
          drivetrainInstance.drive(-adjustedOutput, adjustedOutput);
        } //drivetrainInstance.drive(output, -output)
        // Require driveSubsystem?
    );
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelightInstance;
    this.drivetrain = drivetrainInstance;
    addRequirements(limelight, drivetrain);
    
    this.limelight.enableLEDs();
    
    // Configure additional PID options by calling `getController` here.
    limelight.getPIDController().setTolerance(this.limelight.getThreshold()); // 1 degree for now
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.limelight.disableLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.getPIDController().atSetpoint();
  }
}
