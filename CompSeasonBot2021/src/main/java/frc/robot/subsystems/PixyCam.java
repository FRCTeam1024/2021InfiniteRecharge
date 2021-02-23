// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  private final Pixy2 pixy;
  private final PIDController PID;
  private int pan, tilt;
  boolean isLampEnabled;

  private final double kP, kI, kD;

  /** Creates a new Pixy2. */
  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(0); // Defaults to CS0, inputting 0 it just in case.
    this.setLamp(0);
    pixy.setLED(0, 255, 0);
    
    this.pan = 0;
    this.tilt = 0;
    pixy.setServos(this.pan, this.tilt);

    kP = 0.01;
    kI = 0.0;
    kD = 0.0;

    this.PID = new PIDController(kP, kI, kD);
  }

  public int getBlockCount() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 5);
  }

  public Block getFirstBlock() {
    return pixy.getCCC().getBlockCache().get(0);
  }

  public Block getLargestBlock() {
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 5);
    if(blockCount <= 0) {
      return null;
    }
    //return pixy.getCCC().getBlockCache().get(0);
    ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
    Block largestBlock = null;
    for (Block block : blocks) {
      if(largestBlock == null) {
        largestBlock = block;
      } else if(block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
      }
    }

    return largestBlock;
  }

  public PIDController getPIDController() {
    return this.PID;
  }

  public double getXOffset() {
    return this.getLargestBlock().getX();
  }

  public double getYOffset() {
    return this.getLargestBlock().getY();
  }

  public void setLamp(int state) {
    this.pixy.setLamp((byte) state, (byte) state);
    this.isLampEnabled = (state == 1);
  }

  public void setPan(int pan) {
    this.pixy.setServos(pan, this.tilt);
    this.pan = pan;
  }

  public void setTilt(int tilt) {
    this.pixy.setServos(this.pan, tilt);
    this.tilt = tilt;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Block currentBlock = this.getLargestBlock();
    if (currentBlock != null)
    {
      SmartDashboard.putNumber("Block x", currentBlock.getX());
      SmartDashboard.putNumber("Block y", currentBlock.getY());
      SmartDashboard.putNumber("Block width", currentBlock.getWidth());
      SmartDashboard.putNumber("Block height", currentBlock.getHeight());
    }

    this.setTilt((int) SmartDashboard.getNumber("Servo tilt", 180));
  }
}
