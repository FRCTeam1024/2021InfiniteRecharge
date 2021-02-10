/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX m_LeftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
  private final WPI_TalonSRX m_LeftFollow1 = new WPI_TalonSRX(DriveConstants.kLeftMidMotorPort);
  private final WPI_TalonSRX m_LeftFollow2 = new WPI_TalonSRX(DriveConstants.kLeftRearMotorPort);
  private final WPI_TalonSRX m_RightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
  private final WPI_TalonSRX m_RightFollow1 = new WPI_TalonSRX(DriveConstants.kRightMidMotorPort);
  private final WPI_TalonSRX m_RightFollow2 = new WPI_TalonSRX(DriveConstants.kRightRearMotorPort);
    
  private final Solenoid m_Shift = new Solenoid(DriveConstants.kDrivePCMID, DriveConstants.kDriveSolenoidPort);

  private final double maxVLo;
  private final double maxVHi;
  private final double maxALo;
  private final double maxALoTurn;
  private final double maxAHi;
  private final int sLo;
  private final int sHi;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

    /* Set defaults to avoid unexpected behavior */
    m_LeftLeader.configFactoryDefault();
    m_LeftFollow1.configFactoryDefault();
    m_LeftFollow2.configFactoryDefault();
    m_RightLeader.configFactoryDefault();
    m_RightFollow1.configFactoryDefault();
    m_RightFollow2.configFactoryDefault();

    /*Set neutral mode */
    m_LeftLeader.setNeutralMode(NeutralMode.Brake);
    m_LeftFollow1.setNeutralMode(NeutralMode.Brake);
    m_LeftFollow2.setNeutralMode(NeutralMode.Brake);
    m_RightLeader.setNeutralMode(NeutralMode.Brake);
    m_RightFollow1.setNeutralMode(NeutralMode.Brake);
    m_RightFollow2.setNeutralMode(NeutralMode.Brake);

    /**
     * Configure output direction
     * It would seem that this needs to be done for each controller
     * Followers will not follow the inversion of the leader
     */                                                  
    m_LeftLeader.setInverted(false);      
    m_LeftFollow1.setInverted(false);
    m_LeftFollow2.setInverted(false);
    m_RightLeader.setInverted(true);
    m_RightFollow1.setInverted(true);
    m_RightFollow2.setInverted(true);
    
    /* Set followers to follow */
    m_LeftFollow1.follow(m_LeftLeader); //may want lefts to aux follow right leader instead in order to reduce lag
    m_LeftFollow2.follow(m_LeftLeader);
    m_RightFollow1.follow(m_RightLeader);
    m_RightFollow2.follow(m_RightLeader);

    /* Set Sensor Direction */
    m_LeftLeader.setSensorPhase(false);
    m_RightLeader.setSensorPhase(false); //Seems to affect the final SelectedSensor value only

    /**
     * Closed loop configuration
     */

    /* Configure left side encoder */
    m_LeftLeader.configSelectedFeedbackSensor(  FeedbackDevice.QuadEncoder, // Local Feedback Source
                                                DriveConstants.PID_PRIMARY, // PID Slot for Source
                                                Constants.kTimeoutMs);      // Config Timeout

    /* Configure right side drive to access left encoder value */
    m_RightLeader.configRemoteFeedbackFilter( m_LeftLeader.getDeviceID(),                 // Device ID of Source
                                              RemoteSensorSource.TalonSRX_SelectedSensor, // RemoteFeedback Source
                                              DriveConstants.REMOTE_0,                    // Source Number
                                              Constants.kTimeoutMs);                      // Config Timeout
    
    /* Setup a sum signal to be used for distance PID (Value = Left + right))
     * Using difference terms because local QuadEncoder value is negative in the forward direction
     */
    m_RightLeader.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);  // Feedback device of remote Talon
    m_RightLeader.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);  //or do we want CTRE_MagEncoder_Relative?
    
    /* Setup a difference signal to be used for turn PID (Value = Left - Right) 
     * Using sum terms because local QuadEncoder value is negative in the forward direction
     */
    m_RightLeader.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback device of remote Talon
    m_RightLeader.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);   // Encoder of Current Talon

    /* Set sum signal as selected sensor of distance PID 
    *  Again, using SensorDiffernce because one of the encoders reads negative in the forward direction
    */
    m_RightLeader.configSelectedFeedbackSensor( FeedbackDevice.SensorDifference,
                                                DriveConstants.PID_PRIMARY,
                                                Constants.kTimeoutMs);

    /* Scale primary feedback by 0.5 to return to actual distance since we added the left and right sides 
    */
    m_RightLeader.configSelectedFeedbackCoefficient( 0.5,
                                                     DriveConstants.PID_PRIMARY,
                                                     Constants.kTimeoutMs);

    /* Set difference signal as selected sensor of turn PID 
    *  Again, using SensorSum because one of the encoders reads negative in the forward direction.
    */
    m_RightLeader.configSelectedFeedbackSensor( FeedbackDevice.SensorSum,
                                                DriveConstants.PID_TURN,
                                                Constants.kTimeoutMs);
    
    /* Scale the Feedback Sensor using a coefficient */
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		m_RightLeader.configSelectedFeedbackCoefficient(	DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kEncoderUnitsPerRotation,	// Coefficient
                                                      DriveConstants.PID_TURN, 											                                        // PID Slot of Source
                                                      Constants.kTimeoutMs);														                                    // Configuration Timeout

		/* Set status frame periods, copied form example, gernerally faster than default I think */  
    m_RightLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    m_RightLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    m_RightLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
    m_RightLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		m_LeftLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);		//Used remotely by right Talon, speed up

    /* 1ms per loop. Same as default.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
    m_RightLeader.configClosedLoopPeriod(DriveConstants.kSlot_DistLow, 1, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeriod(DriveConstants.kSlot_TurnLow, 1, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeriod(DriveConstants.kSlot_DistHi, 1, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeriod(DriveConstants.kSlot_TurnHi, 1, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		m_RightLeader.configNeutralDeadband(DriveConstants.kNeutralDeadband, Constants.kTimeoutMs);
		m_LeftLeader.configNeutralDeadband(DriveConstants.kNeutralDeadband, Constants.kTimeoutMs);

		/* max out the peak output (for all modes).  However, you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_LeftLeader.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		m_LeftLeader.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		m_RightLeader.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_RightLeader.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    
    /* Motion profile parameters for low gear */
    maxVLo = 420;  // raw/100ms I think about 420 is about 5 ft/s could probably be faster but works, not sure what max in low gear is.
    maxALo = 210;  // raw/100ms/s 210 Seems good, 
    maxALoTurn = 210;
    sLo = 2;       // Can be 1-8, higher = more smoothing, 2 seems good so far (testing 3 for now)

    /* Motion profile parameters for hi gear */
    maxVHi = 560;  // raw/100ms I think about 10 ft/s could probably be faster, need to test
    maxAHi = 210;  // raw/100ms/s Seems good for lo, we'll see for hi
    sHi = 5;       // Can be 1-8, higher = more smoothing, just a guess so far
    
    /* FPID Gains for Distance PID when in low gear -  Move these to a constants or gains class eventually*/
    m_RightLeader.config_kP(DriveConstants.kSlot_DistLow, 1, Constants.kTimeoutMs);//Increased from 0.6 to 1 to get within 0.5inches
    m_RightLeader.config_kI(DriveConstants.kSlot_DistLow, .0036, Constants.kTimeoutMs);//was 0 increased to .0036 to nudge us within 0.5 inch quicker
    m_RightLeader.config_kD(DriveConstants.kSlot_DistLow, 0, Constants.kTimeoutMs);
    m_RightLeader.config_kF(DriveConstants.kSlot_DistLow, 0, Constants.kTimeoutMs);
    m_RightLeader.config_IntegralZone(DriveConstants.kSlot_DistLow, 100, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeakOutput(DriveConstants.kSlot_DistLow, 1, Constants.kTimeoutMs);
    m_RightLeader.configAllowableClosedloopError(DriveConstants.kSlot_DistLow, 0, Constants.kTimeoutMs);
    //m_RightLeader.configClosedloopRamp(secondsFromNeutralToFull, timeoutMs)

    /* FPID Gains for Distance PID when in hi gear -  Move these to a constants or gains class eventually*/
    m_RightLeader.config_kP(DriveConstants.kSlot_DistHi, 1, Constants.kTimeoutMs);
    m_RightLeader.config_kI(DriveConstants.kSlot_DistHi, 0.0036, Constants.kTimeoutMs);
    m_RightLeader.config_kD(DriveConstants.kSlot_DistHi, 0, Constants.kTimeoutMs);
    m_RightLeader.config_kF(DriveConstants.kSlot_DistHi, 0, Constants.kTimeoutMs);
    m_RightLeader.config_IntegralZone(DriveConstants.kSlot_DistHi, 100, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeakOutput(DriveConstants.kSlot_DistHi, 1, Constants.kTimeoutMs);
    m_RightLeader.configAllowableClosedloopError(DriveConstants.kSlot_DistHi, 0, Constants.kTimeoutMs);
      
    /* FPID Gains for Turn PID when in low gear-  Move these to a constants or gains class eventually*/
    m_RightLeader.config_kP(DriveConstants.kSlot_TurnLow, 0.95, Constants.kTimeoutMs);//was.85 this seems a little better after lowering peak output
    m_RightLeader.config_kI(DriveConstants.kSlot_TurnLow, 0.0036, Constants.kTimeoutMs);
    m_RightLeader.config_kD(DriveConstants.kSlot_TurnLow, 7.5, Constants.kTimeoutMs);
    m_RightLeader.config_kF(DriveConstants.kSlot_TurnLow, 0, Constants.kTimeoutMs);
    m_RightLeader.config_IntegralZone(DriveConstants.kSlot_TurnLow, 200, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeakOutput(DriveConstants.kSlot_TurnLow, .5, Constants.kTimeoutMs); //was .75, lowered to remove jerk during pivot turn
    m_RightLeader.configAllowableClosedloopError(DriveConstants.kSlot_TurnLow, 0, Constants.kTimeoutMs);

    /* FPID Gains for Turn PID when in hi gear-  Move these to a constants or gains class eventually*/
    m_RightLeader.config_kP(DriveConstants.kSlot_TurnHi, 0.5, Constants.kTimeoutMs);
    m_RightLeader.config_kI(DriveConstants.kSlot_TurnHi, 0.0036, Constants.kTimeoutMs);
    m_RightLeader.config_kD(DriveConstants.kSlot_TurnHi, 12, Constants.kTimeoutMs);
    m_RightLeader.config_kF(DriveConstants.kSlot_TurnHi, 0, Constants.kTimeoutMs);
    m_RightLeader.config_IntegralZone(DriveConstants.kSlot_TurnHi, 200, Constants.kTimeoutMs);
    m_RightLeader.configClosedLoopPeakOutput(DriveConstants.kSlot_TurnHi, 1, Constants.kTimeoutMs);
    m_RightLeader.configAllowableClosedloopError(DriveConstants.kSlot_TurnHi, 0, Constants.kTimeoutMs);

    /* configAuxPIDPolarity(boolean invert, int timeoutMs)
    * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
    * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
    */
    m_RightLeader.configAuxPIDPolarity(true, Constants.kTimeoutMs);//was false, maybe would be true if setSensorPhase was flipped

    /* Initialize */
    m_RightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,10); // Copied from example, not sure why this is here.  

    zeroSensors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle", getHeading());
    SmartDashboard.putNumber("RightSidePower", getRightSidePower());
    SmartDashboard.putNumber("LeftSidePower", getLeftSidePower());
    SmartDashboard.putNumber("Distance", getDistance());

  }


  /* Maintained for use with legacy code */
  public void driveForward(double power) {
    // still don't know why we always have to invert this, but whatever...
    drive(-power, -power);    
  }

  /* 
   * Drive command for use with joystick drive and debugging purpose
   * Do not use for auto routines as this will not be repeatable
   */
  public void drive(double leftPower, double rightPower) {

    m_LeftLeader.set(ControlMode.PercentOutput, leftPower);
    m_RightLeader.set(ControlMode.PercentOutput, rightPower);

  }

  /* Maintained for capatability 
   * Recommend using driveTurnLow method
   */
  public void turnRight(double power) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
    drive(-0.00, -power);
  }
  
  /* Maintained for capatability 
   * Recommend using driveTurnLow method
   */
  public void pivotTurnLeftAngle(double leftPower, double rightPower, double targetAngle) {
    // 0.20 is around min power to move
    // set a little power for the inner wheel, makes it easier to turn
    drive(-leftPower, rightPower);
  }

  /* Maintained for capatability 
   * Recommend using driveTurnLow method
   */
  public void pivotTurnRight(double leftPower, double rightPower){
    drive(leftPower, -rightPower);

  }

  /* Maintained for capatability 
   * Recommend using driveTurnLow method
   */
  public void pivotTurnLeft(double leftPower, double rightPower){
    drive(-leftPower, rightPower);

  }

  /* Maintained for capatability 
   * Recommend using driveTurnLow method
   */
  public void turnLeft(double power) {
    // 0.20 is around min power to move
    drive(-power, -0.0);
  }

  public void stop() {
		drive(0.0, 0.0);
  }

  /**
   *  Drive Straight Using Motion Magic within Talon SRX
   * 
   *  @param distance Desired travel distance in inches
   *  @param gear True if high, false if low
   * 
   * */
  public void driveStraight(double distance, boolean gear) {
    
    System.out.println("This is Motion Magic with the Auxiliary PID using the difference between two encoders.\n");
    
    double target_Distance;
    double target_Heading;
    /* Assign parameter slots to PID channels, set Motion Profile Params and shift */
    if (gear) {
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_DistHi, DriveConstants.PID_PRIMARY);
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_TurnHi, DriveConstants.PID_TURN);
      setProfile(maxVHi, maxAHi, sHi);
      shiftHi();
      target_Distance = distance * DriveConstants.kSensorUnitsPerRotation / DriveConstants.kInchesPerRotation;
      target_Heading = 0;

    } else {
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_DistLow, DriveConstants.PID_PRIMARY);
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_TurnLow, DriveConstants.PID_TURN);
      setProfile(maxVLo, maxALo, sLo);
      shiftLow();
      target_Distance = distance * DriveConstants.kSensorUnitsPerRotation / DriveConstants.kInchesPerRotation;
      target_Heading = 0;
    }

    /* Calculate targets */
    
    
    zeroSensors();

    System.out.println("My Distance Target Is: " + target_Distance +"\n");
    System.out.println("My Heading Target Is: " + target_Heading+"\n");

    /* Configure for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
		m_RightLeader.set(ControlMode.MotionMagic, target_Distance, DemandType.AuxPID, target_Heading);
		m_LeftLeader.follow(m_RightLeader, FollowerType.AuxOutput1);
  }

    /**
   *  Turn in Place - Needs improvement, PID needs tuned.  Eventually will want
   *  to use gyro yaw values rather than encoder difference
   * 
   *  @param angle Desired turn angle in degrees 
   *  @param gear True if hi, False if low
   * 
   * */
  public void pivotTurn(double angle, boolean gear) {
    
    System.out.println("This is Motion Magic with the Auxiliary PID using the difference between two encoders.\n");

    /* Assign parameter slots to PID channels, set Motion profile params and shift */
    if (gear) {
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_DistHi, DriveConstants.PID_PRIMARY);
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_TurnHi, DriveConstants.PID_TURN);
      setProfile(maxVHi, maxAHi, sHi);
      shiftHi();
    } else {
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_DistLow, DriveConstants.PID_PRIMARY);
      m_RightLeader.selectProfileSlot(DriveConstants.kSlot_TurnLow, DriveConstants.PID_TURN);
      setProfile(maxVLo, maxALoTurn, sLo);
      shiftLow();
      Timer.delay(0.5);
    }

    /* Calculate targets */
    double target_Distance = 0;
    double target_Heading = angle * DriveConstants.kTurnTravelUnitsPerRotation/360;
    
    zeroSensors();

    System.out.println("My Distance Target Is: " + target_Distance +"\n");
    System.out.println("My Heading Target Is: " + target_Heading+"\n");

    /* Configure for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
		m_RightLeader.set(ControlMode.MotionMagic, target_Distance, DemandType.AuxPID, target_Heading);
		m_LeftLeader.follow(m_RightLeader, FollowerType.AuxOutput1);
  }

  /* Set Gear High */
  public void shiftHi() {
    m_Shift.set(false);
    System.out.println("The drivetrain is in Hi Gear\n");
  }

  /* Set Gear Low */
  public void shiftLow() {
    m_Shift.set(true);
    System.out.println("The drivetrain is in Low Gear\n");
  }

  /* Returns robot heading in degrees 
   * relative to whever it was last zeroed.
  */
  public double getHeading() {
    double a = m_RightLeader.getSelectedSensorPosition(DriveConstants.PID_TURN);
    return a * 360 / DriveConstants.kTurnTravelUnitsPerRotation;
  }

  /* Returns robot distance in inches */
  public double getDistance() {
    double i = m_RightLeader.getSelectedSensorPosition(DriveConstants.PID_PRIMARY); //I wonder how this value compares to getActiveTrajectoryPosition
    return i * DriveConstants.kInchesPerRotation / DriveConstants.kSensorUnitsPerRotation;
  }

  /* The following methods are mostly for debugging purposes only.  They return values in encoder counts
   * It is recommedend that the above methods be used to get values in human units.
   */
  public double getRawAngle() {
    return m_RightLeader.getSelectedSensorPosition(DriveConstants.PID_TURN);
  }

  public double getRawDistance() {
    return m_RightLeader.getSelectedSensorPosition(DriveConstants.PID_PRIMARY);
  }

  public double getLeftRawEncoder() {
    return m_LeftLeader.getSensorCollection().getQuadraturePosition();
  }

  public double getRightRawEncoder() {
    return m_RightLeader.getSensorCollection().getQuadraturePosition();
  }

  public double getRightSidePower(){ //should be giving us the right side's power maybe??? none of its working right so idk
    return m_RightLeader.getMotorOutputPercent();
  }

  public double getLeftSidePower(){
    return m_LeftLeader.getMotorOutputPercent();
  }

  /*
   * Sets up motion magic constants for generating trapezoid/S-curve profile
   * @param maxV Maximum Velocity (420/210)
   * @param maxA Maximum acceleration/deceleration (ramp rate)
   * @param s S curve smoothing factor (1-8)
   * 
   */
  private void setProfile(double maxV, double maxA, int s) {

  m_RightLeader.configMotionCruiseVelocity(maxV, Constants.kTimeoutMs);  
  m_RightLeader.configMotionAcceleration(maxA, Constants.kTimeoutMs); 
  m_RightLeader.configMotionSCurveStrength(s);     

  }

	/* Zero all sensors used */
	public void zeroSensors() {
		m_LeftLeader.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		m_RightLeader.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
	}

  /* 
  All of this is handled in driveStraight or will be handled in the command that calls driveStraight
  public void ForwardInInches(int inches) {
    leftLeader.setSelectedSensorPosition(0);
    inches = (int) (-1 * inches / (6 * 3.14) * 723); //conversion: x inches / (6*PI inches) * 723 encoder units;
    double encoderGoal = leftLeader.getSelectedSensorPosition() + inches;
    while(leftLeader.getSelectedSensorPosition() > inches){
      drive(0.25, 0.25);
    }
    stop();
  }
  public void ReverseInInches(int inches){
    inches = (int) (inches / (6 * 3.14) * 723); 
    double encoderGoal = leftLeader.getSelectedSensorPosition() + inches;
    while(leftLeader.getSelectedSensorPosition() < encoderGoal){
      drive(-0.5, -0.5);
    }
    stop();
  }
  */


  public void LeftArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + m_LeftLeader.getSelectedSensorPosition();
    while(m_LeftLeader.getSelectedSensorPosition() < inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcTurn(){
    double inches = 29.0598;
    inches = (int) (inches / (6 * 3.14) * 723) + m_RightLeader.getSelectedSensorPosition();
    while(m_RightLeader.getSelectedSensorPosition() < inches){
      drive(0.5, 0.22);
    }
    stop();
  }

  public void LeftArcReverse(){
    double inches = 29.0598;
    inches = m_LeftLeader.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(m_LeftLeader.getSelectedSensorPosition() > inches){
      drive(0.22, 0.5);
    }
    stop();
  }

  public void RightArcReverse(){
    double inches = 29.0598;
    inches = m_RightLeader.getSelectedSensorPosition() - ((int) (inches / (6 * 3.14) * 723));
    while(m_RightLeader.getSelectedSensorPosition() > inches){
      drive(0.5, 0.22);
    }
    stop();
  }

  //specific path that is used in part of the Bounce Path.
  //currently empty as it is an optimization that requires turning to an angle.
  public void DiagonalBouncePath(){
    
  }

	// public double getRightEncoderInches() {
  //   return rightLeader.getSelectedSensorPosition();
  //   //wheel diameter: 6 in
  // }
  
	// public void resetEncoders() {
  //   leftLeader.setSelectedSensorPosition(0);
  //   rightLeader.setSelectedSensorPosition(0);
  // }
  
}