// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  private final Pixy2 pixy;
  private final PIDController PID;

  private int pan, tilt;
  
  // These variables are used for detection flicker handling.
  //private Block currentBlock;
  private double xOffset, yOffset, blockWidth, blockHeight; // Last known block properties
  private long lastBlockDetection; // Milliseconds when the last known block was identified
  private Block currentBlock; // The largest block (used to reduce computation)

  boolean isLampEnabled;

  private final double kP, kI, kD;
  public ArrayList<Block> largestBlocks = new ArrayList<Block>();

  /** Creates a new Pixy2. */
  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(0); // Defaults to CS0, inputting 0 just in case.
    setLamp(0);
    
    pan = Constants.PixyConstants.PAN;
    tilt = Constants.PixyConstants.TILT;
    pixy.setServos(pan, tilt);

    lastBlockDetection = 0;
    currentBlock = getLargestBlock();
    xOffset = Constants.PixyConstants.HALF_WIDTH; // Initializes the pixy reading to the center of the screen
    yOffset = Constants.PixyConstants.HALF_HEIGHT; // Half of the vertical height of the pixy

    kP = 0.01;
    kI = 0.0;
    kD = 0.0;

    PID = new PIDController(kP, kI, kD);
  }

  public int getBlockCount() {
    return pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 5);
  }

  public Block getFirstBlock() {
    return pixy.getCCC().getBlockCache().get(0);
  }

  public Block getLargestBlock() {
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 5);
    if(blockCount <= 0) { return null; }

    ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
    Block largestBlock = null;
    
    for (Block block : blocks) {
      if(largestBlock == null) {
        largestBlock = block;
        largestBlocks.add(largestBlock);
      } else if(block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
        largestBlocks.add(largestBlock);
      }
    }
    
    return largestBlock;
  }

  public PIDController getPIDController() {
    return PID;
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getBlockWidth() {
    return blockWidth;
  }

  public double getBlockHeight() {
    return blockHeight;
  }

  public void setLamp(int state) {
    pixy.setLamp((byte) state, (byte) state);
    isLampEnabled = (state == 1);
  }

  public void setPan(int panValue) {
    pixy.setServos(pan, tilt);
    pan = panValue;
  }

  public void setTilt(int tiltValue) {
    pixy.setServos(pan, tilt);
    tilt = tiltValue;
  }

  public int getPan() {
    return pan;
  }

  public int getTilt() {
    return tilt;
  }

  private void calculateOffsets() {
    if(currentBlock != null) {
      lastBlockDetection = System.currentTimeMillis();
      xOffset = currentBlock.getX();
      yOffset = currentBlock.getY();
      blockWidth = currentBlock.getWidth();
      blockHeight = currentBlock.getHeight();
    } else if(System.currentTimeMillis() - lastBlockDetection > 1000) { // If it's been more than 1 second since the last block was found
      xOffset = -1.0;
      yOffset = -1.0;
      blockWidth = -1.0;
      blockHeight = -1.0;
    }
  }

  private void outputBlockData() {
    SmartDashboard.putNumber("Block x", getXOffset());
    SmartDashboard.putNumber("Block y", getYOffset());
    SmartDashboard.putNumber("Block width", getBlockWidth());
    SmartDashboard.putNumber("Block height", getBlockHeight());
    SmartDashboard.putNumber("Pixy target offset", getXOffset() - Constants.PixyConstants.HALF_WIDTH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentBlock = getLargestBlock();
    calculateOffsets();
    outputBlockData();
  }
}
