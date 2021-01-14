/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterPID extends CommandBase {
  private Shooter shooter;
  private CANPIDController pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, targetRPM;

  private ShuffleboardTab shooterTab;
  private NetworkTableEntry pEntry;
  private NetworkTableEntry iEntry;
  private NetworkTableEntry dEntry;
  private NetworkTableEntry izEntry;
  private NetworkTableEntry ffEntry;
  private NetworkTableEntry maxOutputEntry;
  private NetworkTableEntry minOutputEntry;
  private NetworkTableEntry rpmTargetEntry;
  private NetworkTableEntry curVelEntry;

  
  /**
   * Creates a new RunShooterPID.
   */
  public RunShooterPID(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    pidController = shooter.getPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    targetRPM = 5000;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard 
    shooterTab = Shuffleboard.getTab("Shooter");
    pEntry = shooterTab.add("P Gain", kP).getEntry();
    iEntry = shooterTab.add("I Gain", kI).getEntry();
    dEntry = shooterTab.add("D Gain", kD).getEntry();
    izEntry = shooterTab.add("I Zone", kIz).getEntry();
    ffEntry = shooterTab.add("Feed Forward", kFF).getEntry();
    maxOutputEntry = shooterTab.add("Max Output", kMaxOutput).getEntry();
    minOutputEntry = shooterTab.add("Min Output", kMinOutput).getEntry();
    rpmTargetEntry = shooterTab.add("Target RPM", targetRPM).getEntry();

    curVelEntry = shooterTab.add("current RPM", 0).getEntry();

    pidController.setReference(targetRPM, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read PID coefficients from SmartDashboard
    double p = pEntry.getDouble(0);
    double i = iEntry.getDouble(0);
    double d = dEntry.getDouble(0);
    double iz = izEntry.getDouble(0);
    double ff = ffEntry.getDouble(0);
    double max = maxOutputEntry.getDouble(0);
    double min = minOutputEntry.getDouble(0);

    targetRPM = rpmTargetEntry.getDouble(targetRPM);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    //double setoint = m_stick.getY() * maxRPM;
    // pidController.setReference(targetRPM, ControlType.kVelocity);

    curVelEntry.setDouble(shooter.getEncoder().getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
