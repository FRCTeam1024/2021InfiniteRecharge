/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX rightLeader;
  private final WPI_TalonSRX rightFollower1;
  private final WPI_TalonSRX rightFollower2;
  private final WPI_TalonSRX leftLeader;
  private final WPI_TalonSRX leftFollower1;
  private final WPI_TalonSRX leftFollower2;
    
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    rightLeader = new WPI_TalonSRX(20);
    rightFollower1 = new WPI_TalonSRX(11);
    rightFollower2 = new WPI_TalonSRX(4);
    leftLeader = new WPI_TalonSRX(6);
    leftFollower1 = new WPI_TalonSRX(5);
    leftFollower2 = new WPI_TalonSRX(8);

    rightLeader.config_kP(Constants.kSlot_DistLow, Constants.lowGearkP, Constants.kTimeoutMs);
    rightLeader.config_kI(Constants.kSlot_DistLow, Constants.lowGearkI, Constants.kTimeoutMs);
    rightLeader.config_kD(Constants.kSlot_DistLow, Constants.lowGearkD, Constants.kTimeoutMs);

    leftLeader.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
                          Constants.kTimeoutMs);	
                          
    rightLeader.configRemoteFeedbackFilter(leftLeader.getDeviceID(),					// Device ID of Source
                          RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
                          Constants.REMOTE_0,							// Source number [0, 1]
                          Constants.kTimeoutMs);	

    rightLeader.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
    rightLeader.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	
          
    rightLeader.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
                          Constants.kTimeoutMs);
                          
    rightLeader.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
                          Constants.PID_PRIMARY,		// PID Slot of Source 
                          Constants.kTimeoutMs);	
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
    leftLeader.set(ControlMode.PercentOutput, leftPower);
    leftFollower1.set(ControlMode.PercentOutput, leftPower);
    leftFollower2.set(ControlMode.PercentOutput, leftPower);
    rightLeader.set(ControlMode.PercentOutput, -rightPower);
    rightFollower1.set(ControlMode.PercentOutput, -rightPower);
    rightFollower2.set(ControlMode.PercentOutput, -rightPower);
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

  public double getRawEncoder() {
    return rightLeader.getSelectedSensorPosition(Constants.PID_PRIMARY);
    //wheel diameter: 6 in
  }
  
  public void ForwardInInches(int inches){
    leftLeader.setSelectedSensorPosition(0);
    inches = (int) (-1 * inches / (6 * 3.14) * 723); //conversion: x inches / (6*PI inches) * 723 encoder units;
    double encoderGoal = leftLeader.getSelectedSensorPosition() + inches;
    while(leftLeader.getSelectedSensorPosition() > inches){
      drive(0.25, 0.25);
    }
    stop();
  }

  public void ReverseInInches(int inches){
    inches = (int) (inches / (6 * 3.14) * 723); 
    double encoderGoal = leftLeader.getSelectedSensorPosition() + inches;
    while(leftLeader.getSelectedSensorPosition() < encoderGoal){
      drive(-0.5, -0.5);
    }
    stop();
  }

  public void LeftArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + leftLeader.getSelectedSensorPosition();
    while(leftLeader.getSelectedSensorPosition() < inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + rightLeader.getSelectedSensorPosition();
    while(rightLeader.getSelectedSensorPosition() < inches){
      drive(0.5, 0.22);
    }
    stop();
  }

  public void LeftArcReverse(){
    double inches = 29.0598;
    inches = leftLeader.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(leftLeader.getSelectedSensorPosition() > inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcReverse(){
    double inches = 29.0598;
    inches = rightLeader.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(rightLeader.getSelectedSensorPosition() > inches){
      drive(0.5, 0.22);
    }
    stop();
  }


  //specific path that is used in part of the Bounce Path.
  //currently empty as it is an optimization that requires turning to an angle.
  public void DiagonalBouncePath(){
    
  }

	// public double getRightEncoderInches() {
  //   return rightLeader.getSelectedSensorPosition();
  //   //wheel diameter: 6 in
  // }
  
	// public void resetEncoders() {
  //   leftLeader.setSelectedSensorPosition(0);
  //   rightLeader.setSelectedSensorPosition(0);
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
    // SmartDashboard.putNumber("Left Encoder Raw", leftLeader.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder Distance (IN)", -getRightEncoderInches());
    // SmartDashboard.putNumber("Right Encoder Raw", -rightLeader.getSelectedSensorPosition());
  }
}
