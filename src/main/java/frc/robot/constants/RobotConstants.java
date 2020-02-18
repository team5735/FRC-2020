/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import frc.lib.util.Units;

/**
* Add your docs here.
*/
public class RobotConstants {
    public static double JOYSTICK_DEADBAND = 0.1;
    
    /**
     * [––––––FRONT–––––]
     * |                |
     * |                |
     * |                |
     * |                |
     * |                |
     * [______BACK______]
     */

    // Drivetrain Facts
    public static final double ENCODER_TICKS_PER_REV = 2048.0;
    public static final double DRIVETRAIN_TRACK_WIDTH = Units.inchesToMeters(23.0); // m
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0); // m
    
    // Pathfinder Limiting
    public static final double MAX_VELOCITY = 45.0; // m/s, used in generator
    public static final double MAX_VELOCITY_RPM = 5742.0; // RPM
    public static final double kTURN_CORRECTION =  0.0031;
    
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
    public static final double LEFT_kF = 12.0 / MAX_VELOCITY_RPM * ENCODER_TICKS_PER_REV;
    
    public static final double RIGHT_kP = 0.0;
    public static final double RIGHT_kI = 0.0;
    public static final double RIGHT_kD = 0.0;
    public static final double RIGHT_kF = 12.0 / MAX_VELOCITY_RPM * ENCODER_TICKS_PER_REV;
    
    public static final double NORMAL_kP = 0.0;
    public static final double NORMAL_kI = 0.0;
    public static final double NORMAL_kD = 0.0;
    public static final double NORMAL_kF = 12.0 / MAX_VELOCITY_RPM * ENCODER_TICKS_PER_REV;

    public static final double AUTO_LEFT_kP = 1;
    public static final double AUTO_LEFT_kI = 0.0;
    public static final double AUTO_LEFT_kD = 0.0;
    // public static final double AUTO_LEFT_kV = 0.0; // V per rad/s
    // public static final double AUTO_LEFT_kA = 0.0; // V per rad/s^2
    public static final double AUTO_RIGHT_kP = 1;
    public static final double AUTO_RIGHT_kI = 0.0;
    public static final double AUTO_RIGHT_kD = 0.0;
    // public static final double AUTO_RIGHT_kV = 0.0; // V per rad/s
    // public static final double AUTO_RIGHT_kA = 0.0; // V per rad/s^2
    
    // Current Limiting
    public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 38.5, 38.5, 0.25);
    
    // Tuned dynamics
    public static final double ROBOT_LINEAR_INERTIA = 60.0; // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0; // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0; // N*m / (rad/sec) TODO tune
    public static final double DRIVE_VINTERCEPT = 1.055; // V
    
    // Vision Constants
    public static final double CAMERAHEIGHTFROMGROUND = 1;
    public static final double CAMERAANGLEFROMPARALLEL = 0; // Radians
    public static final double TARGETHEIGHTFROMGROUND = 8; // 8 feet and 2 1/4 inches
    
    public static final double CAMERAXOFFSET = 0;
    public static final double CAMERAYOFFSET = 0;
    public static final double CAMERAYAWANGLEDEGREES = 0;
}
