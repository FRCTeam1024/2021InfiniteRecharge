// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * 
 * 
 * Note:  this.XXXX  is only necessary if there is a need to differentiate
 * between a class variable and a variable with a constructor or method of the same name.
 * It is really a matter of preference but it feels over used here.  I think in
 * all cases here the code would be the same without it.  I prefer to just always use
 * different variable names anyway to avoid confusion.
 * 
 */


package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AnglePixy;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  private final Pixy2 pixy;
  private final PIDController PID;

  private int pan, tilt;
  private Block currentBlock;
  boolean isLampEnabled;

  private final double kP, kI, kD;

  /** Creates a new Pixy2. */
  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(0); // Defaults to CS0, inputting 0 it just in case.
    setLamp(0);
    
    pan = 0;
    tilt = 200;
    pixy.setServos(pan, tilt);

    currentBlock = getLargestBlock();

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
      } else if(block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
      }
    }

    return largestBlock;
  }

  public PIDController getPIDController() {
    return PID;
  }

  /**
   * @TODO: Improve detection flicker handling.
   * We need a way for the Pixy to detect if there really is no power cell, or if
   * the detection is just flickering instead.
   * 
   * If the detection is flickering, check again.
   * If there is no power cell, return -1 (no object).
   * 
   * From Dan:
   * See my comments below. I also think we need to be storing the largest block to a private Block object,
   * perhaps called currentBlock, within the Periodic method of this class. If the largest block is ever   
   * null, we just leave currentBlock as is, unless the largest block remains null for an extended time
   * say 0.5-1sec.  Then we know that ball is actually gone and it isn't just flicker.  This method would 
   * be the same except it would return currentBlock.getX() instead of calling getLargestBlock()
   * 
   */ 
  public double getXOffset() {
    Block largestBlock = this.getLargestBlock();
    if(largestBlock != null) { // Implement here a way to compensate for flickering.
      return largestBlock.getX();
    } else { // No power cell, return -1.
      /* Can we establish that getX() is always postive if we see something?
      Then we can return -1 if there is nothing in view and that will tell
      us that no block is currently visible.  We could also use a wrapper
      class for a double value rather than the primitive and then we could 
      return null and just check for a null object from the point where we 
      call this. */

      // This should work but check to make sure
      return -1.0; //return this.getXOffset();
    }
  }

  public double getYOffset() {
    return getLargestBlock().getY();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Block currentBlock = getLargestBlock();
    if (currentBlock != null)
    {
      SmartDashboard.putNumber("Block x", getXOffset());
      SmartDashboard.putNumber("Block y", getYOffset());
      SmartDashboard.putNumber("Block width", currentBlock.getWidth());
      SmartDashboard.putNumber("Block height", currentBlock.getHeight());
    }
  }
}
