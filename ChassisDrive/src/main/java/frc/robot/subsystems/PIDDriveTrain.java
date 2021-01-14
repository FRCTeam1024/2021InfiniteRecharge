/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SerialPort;


public class PIDDriveTrain extends PIDSubsystem {
  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX middleRight;
  private WPI_TalonSRX rearRight;
  public WPI_TalonSRX frontLeft; // This is being used as the encoder
  private WPI_TalonSRX middleLeft;
  private WPI_TalonSRX rearLeft;
  private AnalogGyro navx;
  private I2C i2c;
  public AHRS ahrs;

  private final SimpleMotorFeedforward pidDriveTrainFeedforward =
  new SimpleMotorFeedforward(0.0, 0.0);
  /**
   * Creates a new PIDDriveTrain.
   */
  private final Encoder pidDriveTrainEncoder = new Encoder(0, 1, false);

  public PIDDriveTrain() {
    super(new PIDController(0, 0, 0));
    frontRight = new WPI_TalonSRX(5);

        middleRight = new WPI_TalonSRX(8);

        rearRight = new WPI_TalonSRX(9);

        frontLeft = new WPI_TalonSRX(2);

        middleLeft = new WPI_TalonSRX(3);

        rearLeft = new WPI_TalonSRX(4);

        // private final I2C.Port i2cPort = I2C.Port.kMXP;
        // navx = new AnalogGyro(Port.kMXP);
        ahrs = new AHRS(SerialPort.Port.kMXP);
        getController().setTolerance(0.08);
        pidDriveTrainEncoder.setDistancePerPulse(Robot.driveTrain.getOpticalDistanceInches());
        setSetpoint(0.0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    System.out.println("PIDDriveTrain output : " + output);
    // Use the output here
    // frontLeft.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    // middleLeft.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    // rearLeft.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    
    // frontRight.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    // middleRight.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    // rearRight.setVoltage(output + pidDriveTrainFeedforward.calculate(setpoint));
    double minPower = 0.2;
    double power = output;
    if(output < minPower) {
      power = minPower;
    }
    drive(power, power);
  }

  @Override
  public double getMeasurement() {
    System.out.println("PIDDriveTrain getMeasurement : " + Robot.getLimelightXOffset());
    // Return the process canker variable measurement here
    return Robot.getLimelightXOffset();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void drive(double leftPower, double rightPower) {
    frontLeft.set(ControlMode.PercentOutput, leftPower);
    middleLeft.set(ControlMode.PercentOutput, leftPower);
    rearLeft.set(ControlMode.PercentOutput, leftPower);
    frontRight.set(ControlMode.PercentOutput, -rightPower);
    middleRight.set(ControlMode.PercentOutput, -rightPower);
    rearRight.set(ControlMode.PercentOutput, -rightPower);
  }
}
