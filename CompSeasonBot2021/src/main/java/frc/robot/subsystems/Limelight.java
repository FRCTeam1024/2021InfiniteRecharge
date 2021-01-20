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
  private NetworkTableEntry ledMode;

  /** Creates a new Limelight. */
  public Limelight() {

    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
    this.xOffset = limelight.getEntry("tx");
    this.yOffset = limelight.getEntry("ty");
    this.targetArea = limelight.getEntry("ta");
    this.ledMode = limelight.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getXOffSet() {
    return this.xOffset.getDouble(0.0);
  }

  public NetworkTableEntry getYOffSet() {
    return this.yOffset;
  }

  public NetworkTableEntry getTargetArea() {
    return this.targetArea;
  }

  /**
   * Toggles the LED of the limelight.
   * @param state - 0 for off and 1 for on
   */
  public void toggleLimelightLED(int state) {
    if (state == 0) {
      this.ledMode.setNumber(1);
    } else if (state == 1) {
      this.ledMode.setNumber(3);
    } else {
      return;
    }
  }
}
