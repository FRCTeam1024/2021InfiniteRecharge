// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  private final Pixy2 pixy;
  boolean isLampEnabled = false;

  /** Creates a new Pixy2. */
  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    pixy.setLamp((byte) 0, (byte) 0);
    this.isLampEnabled = false;
    pixy.setLED(0, 255, 0);
  }

  public void setLamp(int state) {
    pixy.setLamp((byte) state, (byte) state);
    this.isLampEnabled = (state == 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
