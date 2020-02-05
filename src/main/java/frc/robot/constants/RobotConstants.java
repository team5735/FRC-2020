/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * Add your docs here.
 */
public class RobotConstants {
    public static double JOYSTICK_DEADBAND = 0.1;
 
    /*
    * ROBOT
    *
    **/

    // Motor Identification
    public static final int COLOR_SPINNER_MOTOR_ID = 1;
    public static final int LEFT_MASTER_ID = 0;
    public static final int RIGHT_MASTER_ID = 1;
    public static final int NORMAL_MASTER_ID = 2;

    // PID
    public static final double LEFT_kP = 0.0;
    public static final double LEFT_kI = 0.0;
    public static final double LEFT_kD = 0.0;
    public static final double LEFT_kF = 0.0;

    public static final double RIGHT_kP = 0.0;
    public static final double RIGHT_kI = 0.0;
    public static final double RIGHT_kD = 0.0;
    public static final double RIGHT_kF = 0.0;

    public static final double NORMAL_kP = 0.0;
    public static final double NORMAL_kI = 0.0;
    public static final double NORMAL_kD = 0.0;
    public static final double NORMAL_kF = 0.0;

    // Wheel stuff for pose
    public static final double DriveWheelTrackWidthInches = 25.54;
    public static final double DriveWheelDiameterInches = 3.92820959548 * 0.99; // TODO TUNE
    public static final double DriveWheelRadiusInches =DriveWheelDiameterInches / 2.0;
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
