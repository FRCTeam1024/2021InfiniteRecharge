/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterPID;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterOne; // this is leader
  private CANSparkMax shooterTwo;
  public CANEncoder shooterEncoderOne;
  private CANEncoder shooterEncoderTwo;

  private CANPIDController pidController;
  
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterOne = new CANSparkMax(39, MotorType.kBrushless);
    shooterTwo = new CANSparkMax(47, MotorType.kBrushless);
    shooterOne.restoreFactoryDefaults();
    shooterTwo.restoreFactoryDefaults();

    pidController = shooterOne.getPIDController();
    shooterTwo.follow(shooterOne, true);
    
    shooterEncoderOne = shooterOne.getEncoder();
    // shooterEncoderTwo = shooterTwo.getEncoder();
    
    setUpShuffleboard();
  }

  public boolean isAtMaxRPM() {
    return shooterEncoderOne.getVelocity() > 5200;
  }

  public boolean isNotAtMaxRPM() {
    return shooterEncoderOne.getVelocity() < 5000;
  }

  public CANEncoder getEncoder() {
    return shooterEncoderOne;
  }

  public CANPIDController getPIDController() {
    return pidController;
  }

  private void setUpShuffleboard() {
    Shuffleboard.getTab("Shooter").add("Run Shooter", new RunShooter(this, 1.0));
    // Shuffleboard.getTab("Shooter").add("Run Shooter PID", new RunShooterPID(this));
    // Shuffleboard.getTab("Shooter").add(this);
  }

  public void runShooterMotors(double motorSpeeds) {
    shooterOne.set(motorSpeeds);
  }

  public void runShooterMotors(double motorOneSpeed, double motorTwoSpeed){
    shooterOne.set(motorOneSpeed);
    shooterTwo.set(motorTwoSpeed);
  }
  
  public void runShooterOne(double motorOneSpeed){
    shooterOne.set(motorOneSpeed);
  }
  
  public void runShooterTwo(double motorTwoSpeed){
    shooterTwo.set(motorTwoSpeed);
  }
  public void stopShooterMotors(){
    shooterOne.set(0.0);
    shooterTwo.set(0.0);
  }
  public void runShooterMotorsUntil(double motorOneSpeed, double motorTwoSpeed, double encoderSetpoint){
    if(shooterEncoderOne.getPosition() < encoderSetpoint && shooterEncoderTwo.getPosition() < encoderSetpoint){
      shooterOne.set(motorOneSpeed);
      shooterTwo.set(motorTwoSpeed);
    } else if(shooterEncoderOne.getPosition() >= encoderSetpoint && shooterEncoderTwo.getPosition() >= encoderSetpoint){
      stopShooterMotors();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Velocity", shooterEncoderOne.getVelocity());
  }
}
