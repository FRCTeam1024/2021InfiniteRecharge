/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ColorsSensors;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.WPIUtilJNI;


public class ControlPannelTurn extends Command {
  private double spinLoops;
  private String requiredColor;
  private int currentLoopAmountHalfs;
  private int detections;
  private double rotationAmount;
  public ControlPannelTurn(String color, double d) {
    // Use requires() here to declare subsystem dependencies
    spinLoops = d;
    requiredColor = color;
    detections = 0;
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    currentLoopAmountHalfs = 0;
    rotationAmount = 0;
    //detections = 0;
    Robot.bitBoardMotors.resetEncoder();
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.bitBoardMotors.drive3WithPower(0.50);
    if(detections < 7) {
      if (ColorsSensors.returnGlobalColorString() == requiredColor) {
          detections++;
      }
    }
    Timer.delay(0.25);
    /**if (ColorsSensors.returnGlobalColorString() == getNeighborColor(requiredColor) && detections == 1) {
      rotationAmount = Robot.bitBoardMotors.getEncoderValue();
      detections++;
      Robot.bitBoardMotors.resetEncoder();
    }**/
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(detections >= 7) {
    /**if(Robot.bitBoardMotors.getEncoderValue() >= (rotationAmount*23)) {
        return true;
      } else {
        return false;
      }**/
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.bitBoardMotors.drive3WithPower(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  
  protected void interrupted() {
  }

 private String getNeighborColor(String color) {
    if(color == "Red") {
      return("Yellow");
    } else if(color == "Yellow") {
      return("Blue");
    } else if(color == "Blue") {
      return("Green");
    } else if(color == "Green") {
      return("Red");
    } else {
      return "Unk";
    }
  }
}
  
