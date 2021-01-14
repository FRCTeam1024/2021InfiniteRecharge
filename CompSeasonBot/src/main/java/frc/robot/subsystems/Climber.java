/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  
  
  public final WPI_TalonSRX hookMotor = new WPI_TalonSRX(3);
  public final WPI_TalonSRX climberMotorLeft = new WPI_TalonSRX(7);
  public final WPI_TalonSRX climberMotorRight = new WPI_TalonSRX(14);
  /**
   * Creates a new Climber.
   */
  public Climber() {
    
  }
  
  public void moveClimber(double leftMotorPower, double rightMotorPower) {
    climberMotorLeft.set(ControlMode.PercentOutput, leftMotorPower);
    climberMotorRight.set(ControlMode.PercentOutput, rightMotorPower);
  }  

  public void moveClimberHook(double motorPower){
    hookMotor.set(ControlMode.PercentOutput, motorPower);
  }  

  public void stopClimber(){
    climberMotorLeft.set(ControlMode.PercentOutput, 0.0);
    climberMotorRight.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
