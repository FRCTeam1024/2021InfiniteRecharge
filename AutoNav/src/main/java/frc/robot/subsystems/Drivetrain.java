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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;


/**
 * Trying to summarize the current issues here so I don't have to go back 
 * and redeploy to find the issues
 * *Error Message: CTR: CAN frame not received/too-stale.
 *      *According to the internet, the issue is that we're not receiving the 
 *        information from the Talons often enough.
 * *Error Message: DifferentialDrive... Output not updated often enough.
 *      *Also an issue with the Talons not giving information quick enough.
 *        Still not sure how we're supposed to actually fix that issue though.
 * *Robot is very jerky when deployed. I'd assume that this has something to
 *    do with the error messages above.
 *      *Multiple people on Chief Delphi have reported issues with robot being 
 *        very jerky to begin with, oftentimes related to the issue with 
 *        Talons not returning information quick enough
 * 
 * All of the problems are coming back to Talons not updating quick enough
 * Most fixes listed on the internet have been related to wiring.
 * However, there could also simply be an issue of how our Encoders are not
 * connected directly to the CAN, as they are in the Trajectory tutorial.
 * Currently trying to work around and find a way to use the encoders as they 
 * are now, connected to the Talon breakout boards.
 */

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX leftEncoderTalon = new WPI_TalonSRX(20);
  private WPI_TalonSRX rightEncoderTalon = new WPI_TalonSRX(6);

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

    // The left-side drive encoder
    leftEncoderTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightEncoderTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    //leftEncoderTalon.setStatusFramePeriod(0, 60); //trying to get rid of timeout error
    //rightEncoderTalon.setStatusFramePeriod(0, 60);
    

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
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getQuadratureVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2, 
                                            m_rightEncoder.getQuadratureVelocity() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2);
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
    outputToSmartDashboard();
    m_odometry.update(navX.getRotation2d(), 
        m_leftEncoder.getQuadraturePosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2, //fudge factor? in testing
        m_rightEncoder.getQuadraturePosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2);
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
    return ((m_leftEncoder.getQuadraturePosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2)
            + (m_rightEncoder.getQuadraturePosition() * Constants.DriveConstants.kMetersPerRotation / Constants.DriveConstants.kSensorUnitsPerRotation / 2)) / 2.0;
  }


    /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setQuadraturePosition(0, 20);
    m_rightEncoder.setQuadraturePosition(0, 20);
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
    //SmartDashboard.putNumber("Pose meters", getPose());
  }
}
