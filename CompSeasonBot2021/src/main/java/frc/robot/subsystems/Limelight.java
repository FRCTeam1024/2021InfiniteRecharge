// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {
  private NetworkTable limelight;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry yOffset;
  private NetworkTableEntry targetArea;

  /** Creates a new Limelight. */
  public Limelight() {

    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
    this.xOffset = limelight.getEntry("tx");
    this.yOffset = limelight.getEntry("ty");
    this.targetArea = limelight.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public NetworkTableEntry getXOffSet() {
    return this.xOffset;
  }

  public NetworkTableEntry getYOffSet() {
    return this.yOffset;
  }

  public NetworkTableEntry getTargetArea() {
    return this.targetArea;
  }
}
