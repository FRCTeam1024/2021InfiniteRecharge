/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {
  /**
   * Creates a new ColorWheel.
   */
  private WPI_TalonSRX colorWheelMotor;

  public ColorWheel() {
    colorWheelMotor = new WPI_TalonSRX(10);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runColorWheel(double motorSpeed){
    colorWheelMotor.set(ControlMode.PercentOutput, motorSpeed);
  }

  public void stopColorWheel(){
    colorWheelMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
