/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

//Robot consistently going 3 times as far as it should go - tried to implement fudge factors, had no effect?

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX leftEncoderTalon = new WPI_TalonSRX(20);
  private WPI_TalonSRX rightEncoderTalon = new WPI_TalonSRX(6);
  private final Solenoid m_Shift = new Solenoid(Constants.DriveConstants.kDrivePCMID, Constants.DriveConstants.kDriveSolenoidPort);

  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(leftEncoderTalon, new WPI_TalonSRX(11), new WPI_TalonSRX(4));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(rightEncoderTalon, new WPI_TalonSRX(5), new WPI_TalonSRX(8));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

// The left-side drive encoder

  SensorCollection m_leftEncoder = new SensorCollection(leftEncoderTalon);
  SensorCollection m_rightEncoder = new SensorCollection(rightEncoderTalon);

  private final AHRS navX;  

  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

    //m_rightMotors.setInverted(true);

    // The left-side drive encoder
    leftEncoderTalon.configFactoryDefault();
    rightEncoderTalon.configFactoryDefault();

    leftEncoderTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 60);
    rightEncoderTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 60);

    leftEncoderTalon.configSelectedFeedbackCoefficient(Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation, 0, 60);
    rightEncoderTalon.configSelectedFeedbackCoefficient(Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation, 0, 60);

    resetEncoders();

    leftEncoderTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 60); //trying to get rid of timeout error
    rightEncoderTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 60);
    

    AHRS a = null;
    try{
      a = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
    navX = a;

    //leftEncoderTalon.setDistancePerPulse(Constants.DriveConstants.kMetersPerRotation);
    //m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kMetersPerRotation);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(navX.getRotation2d().getDegrees()));
    shiftHi();
  }

    public void shiftHi() {
      m_Shift.set(false);
     // System.out.printIn("The drivetrain is in Hi Gear \n");
    } 
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoderTalon.getSelectedSensorVelocity(), 
                                            rightEncoderTalon.getSelectedSensorVelocity());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feedWatchdog();
  }

  

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    //outputToSmartDashboard();   commented out solely because of issues with robot loop overrunning
    m_odometry.update(navX.getRotation2d(), 
        (leftEncoderTalon.getSelectedSensorPosition()),
        (rightEncoderTalon.getSelectedSensorPosition()));
    //may need to scale these based on encoder values vs. circumference of wheel
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navX.getRotation2d());
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getAverageEncoderDistance() {
    return ((-1 * leftEncoderTalon.getSelectedSensorPosition()) + rightEncoderTalon.getSelectedSensorPosition()) / 2.0;
  }


    /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoderTalon.setSelectedSensorPosition(0);
    rightEncoderTalon.setSelectedSensorPosition(0);
  }

  public void driveForward(double power) {
    // still don't know why we always have to invert this, but whatever...
    drive(-power, -power);    
  }

  public void drive(double leftPower, double rightPower) {

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

    
  public void outputToSmartDashboard() {
    // SmartDashboard.putNumber("Left Encoder Distance (IN)", -getLeftEncoderInches());
    // SmartDashboard.putNumber("Left Encoder Raw", frontLeft.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder Distance (IN)", -getRightEncoderInches());
    // SmartDashboard.putNumber("Right Encoder Raw", -frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Raw Left Encoder", leftEncoderTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Raw Right Encoder", rightEncoderTalon.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Pose meters", getPose());
  }
}
