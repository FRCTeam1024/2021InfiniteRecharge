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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final CANSparkMax leadMotor; // this is leader
  private final CANSparkMax followMotor;
  private final CANEncoder shooterEncoder;

  private final CANPIDController shooterPID;

  private final Solenoid hoodSolenoid = new Solenoid(4); //Lets move all device IDs to Constants.java eventually

  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;  //Gains, may move elsewhere.

  //Used to determine if speed has stabilized
  private final double kStableLoops = 10;
  private final double kErrThreshold = 25; //in RPM
  private boolean stable = true;
  private double lastTarget;
  private int withinLoops = 0;

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
    kP = .001; //First guess
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; // From Example, I think 1/max rpm might be what we want but thats an order bigger.
    kMaxOutput = 1; 
    kMinOutput = 0;
    
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);

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

  public void extendHood(){  //These may need their own subsystem to actuate while spinning the shooter
    hoodSolenoid.set(true);
  }
  public void retractHood(){
    hoodSolenoid.set(false);
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
    SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter Stable", isStable());
    SmartDashboard.putNumber("Shooter Power One", getOutput());
    SmartDashboard.putNumber("Shooter Power Two", getOutputTwo());
  }
}
