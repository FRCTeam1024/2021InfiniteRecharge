// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

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
  boolean isLampEnabled;

  /** Creates a new Pixy2. */
  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(0); // Defaults to CS0, inputting 0 it just in case.
    pixy.setLamp((byte) 0, (byte) 0);
    this.isLampEnabled = false;
    pixy.setLED(0, 255, 0);
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

  public void setLamp(int state) {
    pixy.setLamp((byte) state, (byte) state);
    this.isLampEnabled = (state == 1);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Total blocks", this.getBlockCount());
    // This method will be called once per scheduler run
    /*Block currentBlock = this.getFirstBlock();
    SmartDashboard.putNumber("Block x", currentBlock.getX());
    SmartDashboard.putNumber("Block y", currentBlock.getY());
    SmartDashboard.putNumber("Block width", currentBlock.getWidth());
    SmartDashboard.putNumber("Block height", currentBlock.getHeight());*/
  }
}
