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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax leadMotor; // this is leader
  private final CANSparkMax followMotor;
  private final CANEncoder shooterEncoder;

  private final CANPIDController shooterPID;

  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  //Gains, may move elsewhere.

  //Used to determine if speed has stabilized
  private final double kStableLoops = 10;
  private final double kErrThreshold = 100; //in RPM leaving this large until we determine it is needed
  private boolean stable = true;
  private double lastTarget;
  private int withinLoops = 0;

  // For tuning purposes
  private double tuneP;
  private double tuneD;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    
    leadMotor = new CANSparkMax(39, MotorType.kBrushless); //Lets move all device IDs to Constants.java eventually
    followMotor = new CANSparkMax(47, MotorType.kBrushless); //Lets move all device IDs to Constants.java eventually
    
    /* Restore defaults to avoid unexpected problems */
    leadMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    /* Set neutral mode */
    leadMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setIdleMode(IdleMode.kCoast);

    /* Set follower to follow 
     * Use True parameter to invert the follower
     */
    followMotor.follow(leadMotor, true);

    /* Setup PID controller on leader */
    shooterPID = leadMotor.getPIDController();
    
    /* Get access to the encoder */
    shooterEncoder = leadMotor.getEncoder();

    /*
     * Setting Gains here, for tuning, override these using Spark MAX GUI, then hard code here
     * once we settle on values.  Consider moving these to Constants.java eventually
     */
    kP = .0012;       // .0012 test value is working well, may need to get higher to reduce droop
    kI = 0;
    kD = 0.07;        // 0.07 test value is working well 
    kIz = 0; 
    kFF = 0.0002;     // 1/4900 (tested max rpm)
    kMaxOutput = 1; 
    kMinOutput = 0;
    
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);

    // For tuning purposes (remove once tuning is finished.)
    SmartDashboard.putNumber("Shoot P", leadMotor.getPIDController().getP());
    SmartDashboard.putNumber("Shoot D", leadMotor.getPIDController().getD());
    tuneP = SmartDashboard.getNumber("Shoot P", kP);
    tuneD = SmartDashboard.getNumber("Shoot D", kD);
  }

  /*
   * Used to run the shooter under PID control
   * 
   * @param speed Desired speed in RPM
   * 
   */
  public void runControlledShooter(double speed) {
    lastTarget = speed;
    shooterPID.setReference(speed, ControlType.kVelocity);
  }

  /* simple method to run motors at percent power 
   * maintained for capatibility with legacy code and for troubleshooting
   * Do not use for new code
   */
  public void runShooterMotors(double power) {
    leadMotor.set(power);
  }
  
  public void stopShooterMotors(){
    lastTarget = 0;
    leadMotor.set(0.0);
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  // Created for compatibilty with existing code, may want to do this differently
  public boolean isStable(){
    return(withinLoops > kStableLoops);
  }

  // Created for compatibility with existing code, may want to do this differently
  public boolean isNotStable(){
    return !isStable();
  }

  // The motor controller's applied output duty cycle.
  // Used for tuning the ShooterPID
  public double getOutput() {
    return leadMotor.getAppliedOutput();
  }

  public double getOutputTwo() {
    return followMotor.getAppliedOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Keep track of if the speed is stable and for how long it has been
    if (getShooterSpeed() - lastTarget < +kErrThreshold &&
        getShooterSpeed() - lastTarget > -kErrThreshold) {
        
          ++withinLoops;
    }
    else {
      withinLoops = 0;
    }

    //Put some debug info to the dashboard
    SmartDashboard.putNumber("Shooter Velocity", getShooterSpeed());
    SmartDashboard.putBoolean("Shooter Stable", isStable());
    // For tuning purposes (delete when tuning is finished.)
    if(SmartDashboard.getNumber("Shoot P", kP) != tuneP && tuneP > 0.0 && tuneP <= 0.1) {
      tuneP = SmartDashboard.getNumber("Shoot P", kP);
      leadMotor.getPIDController().setP(tuneP);
    }
    if(SmartDashboard.getNumber("Shoot D", kD) != tuneD && tuneD >= 0.0 && tuneD < 1.0) {
      tuneD = SmartDashboard.getNumber("Shoot D", kD);
      leadMotor.getPIDController().setD(tuneD);
    }
    //SmartDashboard.putNumber("Shooter Power One", getOutput());
    //SmartDashboard.putNumber("Shooter Power Two", getOutputTwo());
  }
}
