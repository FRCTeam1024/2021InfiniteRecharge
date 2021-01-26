// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Library for viewing the cameras connected to the RoboRIO
import edu.wpi.first.cameraserver.CameraServer;

public class Pixy2 extends SubsystemBase {
  /** Creates a new Pixy2. */
  public Pixy2() {
    // Get the camera server
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
