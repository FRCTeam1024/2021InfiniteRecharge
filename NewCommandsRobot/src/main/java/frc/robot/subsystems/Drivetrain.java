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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX middleRight;
  private WPI_TalonSRX rearRight;
  public WPI_TalonSRX frontLeft; // This is being used as the encoder
  private WPI_TalonSRX middleLeft;
  private WPI_TalonSRX rearLeft;
  public AHRS ahrs;
    
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    frontRight = new WPI_TalonSRX(5);
    middleRight = new WPI_TalonSRX(8);
    rearRight = new WPI_TalonSRX(9);
    frontLeft = new WPI_TalonSRX(2);
    middleLeft = new WPI_TalonSRX(3);
    rearLeft = new WPI_TalonSRX(4);

    // ahrs = new AHRS(SerialPort.Port.kMXP); // when using the wide cable or sitting directly on rio
    // ahrs = new AHRS(SerialPort.Port.kUSB1);
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

  public void setBrakeMode() {
    frontLeft.setNeutralMode(NeutralMode.Brake);
    middleLeft.setNeutralMode(NeutralMode.Brake);
    rearLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    middleRight.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    frontLeft.setNeutralMode(NeutralMode.Coast);
    middleLeft.setNeutralMode(NeutralMode.Coast);
    rearLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    middleRight.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
  }

  public void turnRight(double power) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
    drive(-0.00, -power);
  }
  
  public void pivotTurnRightAngle(double leftPower, double rightPower, double targetAngle) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
   // ahrs.reset();
    if(ahrs.getYaw() < targetAngle){
      drive(leftPower, -rightPower);
    } else if(ahrs.getYaw() >= targetAngle){
      drive(0.0, 0.0);
    }
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

  public double getLeftEncoderInches() {
    return frontLeft.getSelectedSensorPosition() * 1/70; //(1.0 / 71.0) * 4.0;
    //wheel diameter: 6 in
  }
  
	public double getRightEncoderInches() {
    return frontRight.getSelectedSensorPosition() * 1/70; //(1.0 / 71.0) * 4.0;
    //wheel diameter: 6 in
  }
  
	public void resetEncoders() {
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
  }

  public float getHeading() {
      // put it behind a method for now because we're not sure if we're using yaw, if the
      // navx is vertical, or compassHeading, if the navx is flat/horizontal

      // if it's yaw, may have to do some calculations, if yaw is in range of -180 to 180, 
      // instead of 0 - 360
      // return ahrs.getYaw();
      return 0;
  }

  public void resetGyro() {
    ahrs.reset(); // not sure what this does
  }
    
  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("Left Encoder Distance (IN)", -getLeftEncoderInches());
    SmartDashboard.putNumber("Left Encoder Raw", -frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder Distance (IN)", -getRightEncoderInches());
    SmartDashboard.putNumber("Right Encoder Raw", -frontRight.getSelectedSensorPosition());
    // SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
    // SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
  }
}
