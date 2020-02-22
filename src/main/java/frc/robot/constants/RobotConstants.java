/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

/**
 * Add your docs here.
 */
public class RobotConstants {
    public static double JOYSTICK_DEADBAND = 0.1;
 
    /*
    * ROBOT
    *
    **/

    // Robot Constants
    public static final double GearRatio = 12.0 / 64.0 * 24.0 / 28.0;
    public static final double EncoderTicksPerRotation = 2048.0 / GearRatio;
    public static final double MaxVelocity = 1.0; // m/s

    // Motor Identification
    public static final int COLOR_SPINNER_MOTOR_ID = 1;
    public static final int LEFT_MASTER_ID = 57;
    public static final int LEFT_SLAVE_ID = 56;

    public static final int RIGHT_MASTER_ID = 58;
    public static final int RIGHT_SLAVE_ID = 59;

    public static final int NORMAL_ID = 55;
    
    public static final int GYRO_TALON_HOST_ID = 4; //TODO tune

    // PID
    public static final double LEFT_kP = 0.0;
    public static final double LEFT_kI = 0.0;
    public static final double LEFT_kD = 0.0;
    public static final double LEFT_kF = 12.0 / 5742.0 * 2048.0;

    public static final double RIGHT_kP = 0.0;
    public static final double RIGHT_kI = 0.0;
    public static final double RIGHT_kD = 0.0;
    public static final double RIGHT_kF = 12.0 / 5742.0 * 2048.0;

    public static final double NORMAL_kP = 0.0;
    public static final double NORMAL_kI = 0.0;
    public static final double NORMAL_kD = 0.0;
    public static final double NORMAL_kF = 12.0 / 5742.0 * 2048.0;

    public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 38.5, 38.5, 0.25);


    // Wheel stuff for pose
    public static final double DriveWheelTrackWidthInches = 23.0;
    public static final double DriveWheelDiameterInches = 6.0; // TODO TUNE
    public static final double DriveWheelRadiusInches = DriveWheelDiameterInches / 2.0;
    public static final double TrackScrubFactor = 1.0;

     // Tuned dynamics
     public static final double RobotLinearInertia = 60.0; // kg TODO tune
     public static final double RobotAngularInertia = 10.0; // kg m^2 TODO tune
     public static final double RobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
     public static final double DriveVIntercept = 1.055; // V
     public static final double DriveKv = 0.135; // V per rad/s
     public static final double DriveKa = 0.012; // V per rad/s^2
 

    // Vision Constants
    public static final double CAMERAHEIGHTFROMGROUND = 1;
    public static final double CAMERAANGLEFROMPARALLEL = 0; // Radians
    public static final double TARGETHEIGHTFROMGROUND = 8; // 8 feet and 2 1/4 inches

    public static final double CAMERAXOFFSET = 0;
    public static final double CAMERAYOFFSET = 0;
    public static final double CAMERAYAWANGLEDEGREES = 0;
}
