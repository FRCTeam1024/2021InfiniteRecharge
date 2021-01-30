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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

  public WPI_TalonSRX frontRight;
  private WPI_TalonSRX middleRight;
  private WPI_TalonSRX rearRight;
  public WPI_TalonSRX frontLeft; // This is being used as the encoder
  private WPI_TalonSRX middleLeft;
  private WPI_TalonSRX rearLeft;
    
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    frontRight = new WPI_TalonSRX(20);
    middleRight = new WPI_TalonSRX(11);
    rearRight = new WPI_TalonSRX(4);
    frontLeft = new WPI_TalonSRX(6);
    middleLeft = new WPI_TalonSRX(5);
    rearLeft = new WPI_TalonSRX(8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    outputToSmartDashboard();
  }

  public void driveForward(double power) {
    // still don't know why we always have to invert this, but whatever...
    drive(-power, -power);    
  }

  public void drive(double leftPower, double rightPower) {
    frontLeft.set(ControlMode.PercentOutput, leftPower);
    middleLeft.set(ControlMode.PercentOutput, leftPower);
    rearLeft.set(ControlMode.PercentOutput, leftPower);
    frontRight.set(ControlMode.PercentOutput, -rightPower);
    middleRight.set(ControlMode.PercentOutput, -rightPower);
    rearRight.set(ControlMode.PercentOutput, -rightPower);
  }

  public void turnRight(double power) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
    drive(-0.00, -power);
  }
  
  public void pivotTurnLeftAngle(double leftPower, double rightPower, double targetAngle) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
    drive(-leftPower, rightPower);
  }
  
  public void pivotTurnRight(double leftPower, double rightPower){
    drive(leftPower, -rightPower);

  }
  
  public void pivotTurnLeft(double leftPower, double rightPower){
    drive(-leftPower, rightPower);

  }

  public void turnLeft(double power) {
    // 0.20 is around min power to move
    drive(-power, -0.0);
  }

  public void stop() {
		drive(0.0, 0.0);
  }

  public double getRawLeftEncoder() {
    return frontLeft.getSelectedSensorPosition();
    //wheel diameter: 6 in
  }
  
  public void ForwardInInches(int inches){
    frontLeft.setSelectedSensorPosition(0);
    inches = (int) (-1 * inches / (6 * 3.14) * 723); //conversion: x inches / (6*PI inches) * 723 encoder units;
    double encoderGoal = frontLeft.getSelectedSensorPosition() + inches;
    while(frontLeft.getSelectedSensorPosition() > inches){
      drive(0.25, 0.25);
    }
    stop();
  }

  public void ReverseInInches(int inches){
    inches = (int) (inches / (6 * 3.14) * 723); 
    double encoderGoal = frontLeft.getSelectedSensorPosition() + inches;
    while(frontLeft.getSelectedSensorPosition() < encoderGoal){
      drive(-0.5, -0.5);
    }
    stop();
  }

  public void LeftArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + frontLeft.getSelectedSensorPosition();
    while(frontLeft.getSelectedSensorPosition() < inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + frontRight.getSelectedSensorPosition();
    while(frontRight.getSelectedSensorPosition() < inches){
      drive(0.5, 0.22);
    }
    stop();
  }

  public void LeftArcReverse(){
    double inches = 29.0598;
    inches = frontLeft.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(frontLeft.getSelectedSensorPosition() > inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcReverse(){
    double inches = 29.0598;
    inches = frontRight.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(frontRight.getSelectedSensorPosition() > inches){
      drive(0.5, 0.22);
    }
    stop();
  }


  //specific path that is used in part of the Bounce Path.
  //currently empty as it is an optimization that requires turning to an angle.
  public void DiagonalBouncePath(){
    
  }

	// public double getRightEncoderInches() {
  //   return frontRight.getSelectedSensorPosition();
  //   //wheel diameter: 6 in
  // }
  
	// public void resetEncoders() {
  //   frontLeft.setSelectedSensorPosition(0);
  //   frontRight.setSelectedSensorPosition(0);
  // }

  public float getHeading() {
      // put it behind a method for now because we're not sure if we're using yaw, if the
      // navx is vertical, or compassHeading, if the navx is flat/horizontal

      // if it's yaw, may have to do some calculations, if yaw is in range of -180 to 180, 
      // instead of 0 - 360
      // return ahrs.getYaw();
      return 0;
  }
    
  public void outputToSmartDashboard() {
    // SmartDashboard.putNumber("Left Encoder Distance (IN)", -getLeftEncoderInches());
    // SmartDashboard.putNumber("Left Encoder Raw", frontLeft.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder Distance (IN)", -getRightEncoderInches());
    // SmartDashboard.putNumber("Right Encoder Raw", -frontRight.getSelectedSensorPosition());
  }
}
