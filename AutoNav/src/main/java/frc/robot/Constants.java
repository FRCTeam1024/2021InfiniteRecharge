/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

 /**
     * Controller initialization timeout duration
     * set to 0 to skip waiting for confirmation
     */
    public static final int kTimeoutMs = 60;    //example was 30 made larger because was timing out, may be able to go back

    //public static final int QuadEncoderPortA = 7; //encoder ports found at the TalonSRX encoder users manual
    //public static final int QuadEncoderPortB = 5; Not in use as they were needed for the Encoder class, which does not work with our encoders
    public static final boolean kLeftEncoderReversed = false; //assumed bc our drive function is reversed
    public static final boolean kRightEncoderReversed = true;
    public static final double PI = 3.14159;
    
   

    public static final class DriveConstants {
        public static final double ksVolts = 1; //in characterization data: 1.31  Testing 1
        public static final double kvVoltSecondsPerMeter = 1; //in characterization data: 1.95  Testing 1
        public static final double kaVoltSecondsSquaredPerMeter = 0.25; //in characterization data: 0.765  Testing 0.25 for slower acceleration
        public static final double kPDriveVel = 1; //in characterization data: 2.91  Testing 1 to try and slow down the robot

        public static final double kTrackwidthMeters = 0.59;  //characterized 1.32; measured 0.59
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 2.5; //these may have to be changed 
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5; //max of 3 for both of these values, changed for testing

        public static final double kRamseteB = 2; //may need to be tuned
        public static final double kRamseteZeta = 0.7;

        /* CAN IDs   */
        public static final int kLeftRearMotorPort = 8;
        public static final int kLeftMidMotorPort = 5;
        public static final int kLeftFrontMotorPort = 6;
        public static final int kRightRearMotorPort = 4;
        public static final int kRightMidMotorPort = 11;
        public static final int kRightFrontMotorPort = 20;

        /* PCM ID */
        public static final int kDrivePCMID = 0;

        /* Solenoid Channels*/
        public static final int kDriveSolenoidPort = 1;

        /* Desired left-right units difference for one 360 deg rotation */
        public static final double kTurnTravelUnitsPerRotation = 3600;

        /* Actual left-right encoder difference for one 360 deg rotation of the robot */
        public static final int kEncoderUnitsPerRotation = 11587;  //Tested and confirmed this value for 2021 compbot

        /* Encoder Counts Per Wheel Rotation (3:1 ratio in AM Gearbox)*/
        public static final double kSensorUnitsPerRotation = 1365.3333; //testing to see if it lowering this for some unknown reason would make it work???
    


        /* Inches per wheel rotation */
        public static final double kInchesPerRotation = 19.0805; //Tested and confirmed this value for 2021 compbot

        //meters per wheel rotation - used for trajectory class
        public static final double kMetersPerRotation = 0.4846447;
        
        /* Distance per rotation of encoder in meters */
        public static final double kEncoderDistancePerPulse = 1.454; // 3 rotations of wheel in meters - not quite sure if math is corect, may need to fix later

        /**
	    * Motor neutral dead-band, set to the minimum 0.1%.
	    */
        public static final double kNeutralDeadband = 0.001; //might think about increasing. Is it ok for motors to sit stalled at low power?

        /* Remote device ordinals */
        public static final int REMOTE_0 = 0;
        public static final int REMOTE_1 = 1;

        /* PID Indices */
        public static final int PID_PRIMARY = 0;
        public static final int PID_TURN = 1;

        /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	    public static final int SLOT_0 = 0;
	    public static final int SLOT_1 = 1;
	    public static final int SLOT_2 = 2;
        public final static int SLOT_3 = 3;
        
	    /* ---- Named slots, used to clarify code ---- */
	    public static final int kSlot_DistLow = SLOT_0;
	    public static final int kSlot_TurnLow = SLOT_1;
	    public static final int kSlot_DistHi = SLOT_2;
        public static final int kSlot_TurnHi = SLOT_3;
    }

}