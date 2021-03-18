/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private CANEncoder intakeEncoder;
  private CANSparkMax intakeMotor;

  // Was 1
  //testing it on port 2
  private final Solenoid intakeSolenoid = new Solenoid(0);

  public Intake() {
    intakeMotor = new CANSparkMax(24, MotorType.kBrushless);
    //intakeEncoder = new CANEncoder(intakeMotor);
    intakeEncoder = intakeMotor.getEncoder();
  }

  public void runIntake(double motorSpeed){
    intakeMotor.set(motorSpeed);
  }
  public void stopIntake(){
    intakeMotor.set(0.0);
  }
  public void extendIntake(){
    intakeSolenoid.set(false);
  }
  public void retractIntake(){
     intakeSolenoid.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (intakeMotor.get() != 0.0) {
      System.out.println(intakeMotor.get());
    }
  }
}
