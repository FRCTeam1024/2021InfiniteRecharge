// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private NetworkTable limelight;
  private NetworkTableEntry targetEntry;
  private NetworkTableEntry xOffsetEntry;
  private NetworkTableEntry yOffsetEntry;
  private NetworkTableEntry targetAreaEntry;
  private NetworkTableEntry ledModeEntry;

  private final PIDController limelightPID;
  private final double kP, kI, kD, kMaxOutput, kMinOutput;  //Gains, may move elsewhere.
  //private final double kIz, kFF;

  /** Creates a new Limelight. */
  public Limelight() {
    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
    this.limelight.getEntry("camMode").setNumber(1);

    this.targetEntry = limelight.getEntry("tv");
    this.xOffsetEntry = limelight.getEntry("tx");
    this.yOffsetEntry = limelight.getEntry("ty");
    this.targetAreaEntry = limelight.getEntry("ta");
    this.ledModeEntry = limelight.getEntry("ledMode");
    
    kP = 0.03;
    kI = 0;
    kD = 0; 
    //kIz = 0;  // Not used for PIDController class
    //kFF = 0.1;  // Not used for PIDController class
    this.kMaxOutput = 1.0;
    this.kMinOutput = -1.0;

    this.limelightPID = new PIDController(kP, kI, kD);

    SmartDashboard.putNumber("LkP", this.limelightPID.getP());
    SmartDashboard.putNumber("LkI", this.limelightPID.getI());
    SmartDashboard.putNumber("LkD", this.limelightPID.getD());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public PIDController getPIDController() {
    this.limelightPID.setP(SmartDashboard.getNumber("LkP", 0));
    this.limelightPID.setI(SmartDashboard.getNumber("LkI", 0));
    this.limelightPID.setD(SmartDashboard.getNumber("LkD", 0));
    return this.limelightPID;
  }

  public double getMinOutput() {
    return this.kMinOutput;
  }

  public double getMaxOutput() {
    return this.kMaxOutput;
  }

  public double getXOffset() {
    return this.xOffsetEntry.getDouble(0.0);
  }

  public double getYOffset() {
    return this.yOffsetEntry.getDouble(0.0);
  }

  public double getTargetArea() {
    return this.targetAreaEntry.getDouble(0.0);
  }

  public boolean hasTarget() {
    // defaults to false to negate false-positives
    return this.targetEntry.getBoolean(false);
  }

  public void enableLEDs() {
    this.ledModeEntry.setNumber(3);
  }

  public void disableLEDs() {
    this.ledModeEntry.setNumber(1);
  }

  // Toggle LEDs based on current state
  /*public void toggleLEDs() {

  }*/

  /**
   * Toggles the LED of the limelight.
   * @param state - 0 for off and 1 for on
   */
  public void toggleLEDs(int state) {
    if (state == 0) {
      this.ledModeEntry.setNumber(1);
    } else if (state == 1) {
      this.ledModeEntry.setNumber(3);
    } else {
      return;
    }
  }
}
