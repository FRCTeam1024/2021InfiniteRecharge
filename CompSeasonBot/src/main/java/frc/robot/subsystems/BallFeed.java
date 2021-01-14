/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallFeed extends SubsystemBase {
  /**
   * Creates a new BallFeed.
   */
  private WPI_TalonSRX ballFeedMotor; // this is the big white lower one
  private WPI_TalonSRX shooterFeedMotor; // this is the small blue one


  public BallFeed() {
//ballfeed motor 9
ballFeedMotor = new WPI_TalonSRX(19);
shooterFeedMotor = new WPI_TalonSRX(13);

//shooterfeed motor 13
  }

  public void runBallFeedMotor(double motorSpeed){
    ballFeedMotor.set(motorSpeed);
  }
  public void stopBallFeedMotor(){
    ballFeedMotor.set(0.0);
  }
  public void runShooterFeedMotor(double motorSpeed){
    shooterFeedMotor.set(motorSpeed);
  }
  public void stopShooterFeedMotor(){
    shooterFeedMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
