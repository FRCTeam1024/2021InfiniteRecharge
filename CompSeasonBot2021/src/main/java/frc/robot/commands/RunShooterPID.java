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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterPID extends CommandBase {
  private Shooter shooter;
  private double targetSpeed;
  private CANPIDController pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM; // (replaced with targetSpeed param)

  /*private ShuffleboardTab shooterTab;
  private NetworkTableEntry pEntry;
  private NetworkTableEntry iEntry;
  private NetworkTableEntry dEntry;
  private NetworkTableEntry izEntry;
  private NetworkTableEntry ffEntry;
  private NetworkTableEntry maxOutputEntry;
  private NetworkTableEntry minOutputEntry;
  //private NetworkTableEntry rpmTargetEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry curVelEntry;*/
  
  /**
   * Creates a new RunShooterPID.
   * targetSpeed equates to the target shooter speed (-1.0 to 1.0)
   * 	For shooting forwards, keep this value between 0.0 and 1.0. Ideally this should be slightly lower than 1.0 to allow for increased speed.
   */
  public RunShooterPID(Shooter shooter, double targetSpeed) {
    this.shooter = shooter;
    this.targetSpeed = targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    pidController = shooter.getPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // PID coefficients
    kP = 0.001; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = 0; // Was -1, we want between 0 and 1 (forwards)
    maxRPM = 5676;
    targetRPM = targetSpeed * maxRPM; // Was 5000, 5676 is empirical max RPM

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    pidController.setReference(targetRPM, ControlType.kVelocity);

    // display PID coefficients on SmartDashboard 
    /*shooterTab = Shuffleboard.getTab("Shooter");
    pEntry = shooterTab.add("P Gain", kP).getEntry();
    iEntry = shooterTab.add("I Gain", kI).getEntry();
    dEntry = shooterTab.add("D Gain", kD).getEntry();
    izEntry = shooterTab.add("I Zone", kIz).getEntry();
    ffEntry = shooterTab.add("Feed Forward", kFF).getEntry();
    maxOutputEntry = shooterTab.add("Max Output", kMaxOutput).getEntry();
    minOutputEntry = shooterTab.add("Min Output", kMinOutput).getEntry();
    //rpmTargetEntry = shooterTab.add("Target RPM", targetRPM).getEntry();
    targetSpeedEntry = shooterTab.add("Target Speed", targetSpeed).getEntry();
    curVelEntry = shooterTab.add("current Speed", 0).getEntry();*/

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    //pidController.setReference(targetSpeed, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read PID coefficients from SmartDashboard
    /*double p = pEntry.getDouble(0);
    double i = iEntry.getDouble(0);
    double d = dEntry.getDouble(0);
    double iz = izEntry.getDouble(0);
    double ff = ffEntry.getDouble(0);
    double max = maxOutputEntry.getDouble(0);
    double min = minOutputEntry.getDouble(0);

    //targetRPM = rpmTargetEntry.getDouble(targetRPM);
    targetSpeed = targetSpeedEntry.getDouble(targetSpeed);

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
    curVelEntry.setDouble(shooter.getEncoder().getVelocity());
    */

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    /*double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);*/

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
    
    double setPoint, processVariable;
    setPoint = SmartDashboard.getNumber("Set Velocity", 0.5 * maxRPM);
    pidController.setReference(setPoint, ControlType.kVelocity);
    processVariable = shooter.getEncoder().getVelocity();
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", shooter.getOutput());

    SmartDashboard.putNumber("Encoder One", shooter.getOutput());
    SmartDashboard.putNumber("Encoder Two", shooter.getOutput());
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
