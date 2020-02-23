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
    public static final double DRIVETRAIN_GEAR_RATIO = 6.22222222 / 1.0; // for every 6.2 rev of Falcon, wheels turn 1 rev
    public static final double ENCODER_TICKS_PER_FALCON_REV = 2048.0;
    public static final double ENCODER_TICKS_PER_WHEEL_REV = ENCODER_TICKS_PER_FALCON_REV * DRIVETRAIN_GEAR_RATIO;
    public static final double DRIVETRAIN_TRACK_WIDTH = Units.inchesToMeters(23.0); // m
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6.0); // m
    
    // Pathfinder Limiting
    public static final double MAX_VELOCITY_TICKS = 10500.0; // ticks / 100ms
    public static final double MAX_VELOCITY_RPM = 496.156; // MAX_VELOCITY_TICKS * 10 * 60 / 2048 / DRIVETRAIN_GEAR_RATIO; // RPM, with gear ratio in account
    public static final double MAX_VELOCITY = 3.896; // MAX_VELOCITY_TICKS * 10 / 2048 / DRIVETRAIN_GEAR_RATIO * (Math.PI * 0.15) // m/s, used in generator
    public static final double kTURN_CORRECTION = -0.009;
    
    // Motor Identification
    public static final int COLOR_SPINNER_MOTOR_ID = 1;
    public static final int LEFT_MASTER_ID = 57;
    public static final int LEFT_SLAVE_ID = 56;
    
    public static final int RIGHT_MASTER_ID = 58;
    public static final int RIGHT_SLAVE_ID = 59;
    
    public static final int NORMAL_ID = 55;

    public static final int GYRO_TALON_HOST_ID = 6;

    public static final int WINCH_ID = 40;
    
    // PID
    public static final double LEFT_kP = 0.0;
    public static final double LEFT_kI = 0.0;
    public static final double LEFT_kD = 0.0;
    public static final double LEFT_kF = 0.064;
    
    public static final double RIGHT_kP = 0.0;
    public static final double RIGHT_kI = 0.0;
    public static final double RIGHT_kD = 0.0;
    public static final double RIGHT_kF = 0.0723;
    
    public static final double NORMAL_kP = 0.0;
    public static final double NORMAL_kI = 0.0;
    public static final double NORMAL_kD = 0.0;
    public static final double NORMAL_kF = ENCODER_TICKS_PER_FALCON_REV / MAX_VELOCITY_TICKS;

    public static final double AUTO_LEFT_kP = 0.0;//0.018;
    public static final double AUTO_LEFT_kI = 0.0;
    public static final double AUTO_LEFT_kD = 0.0;
    public static final double AUTO_RIGHT_kP = 0.0;//0.018;
    public static final double AUTO_RIGHT_kI = 0.0;
    public static final double AUTO_RIGHT_kD = 0.0;

    public static final double FLYWHEEL_kP = 0.0;//0.000325;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;//0.004;
    public static final double FLYWHEEL_kF = 0.7 / 4900.0;
    
    // Flywheel
    public static final double FLYWHEEL_PULLEY_RATIO = 62.0 / 36.0;
    public static final double FLYWHEEL_MIN_SPEED = 3500; // rpm
    public static final double FLYWHEEL_MAX_SPEED = 5800; // rpm
    public static final double FLYWHEEL_PRESET_LINE = 3500; // rpm
    public static final double FLYWHEEL_PRESET_TRENCH = 3550; // rpm
    public static final double FLYWHEEL_PRESET_BEHINDCOLORWHEEL = 4500; // rpm

    // Intake
    public static final double INTAKE_kP = 0.000325;
    public static final double INTAKE_kI = 0.0;
    public static final double INTAKE_kD = 0.004;
    public static final double INTAKE_POSITION_DEADBAND = 10; // sensor units
    public static final double INTAKE_POSITION_RETRACTED = 100;
    public static final double INTAKE_POSITION_DEPLOYED = 4500;

    // Current Limiting
    public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 38.5, 38.5, 0.25);
    
    // Tuned dynamics
    public static final double ROBOT_LINEAR_INERTIA = 60.0; // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0; // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0; // N*m / (rad/sec) TODO tune
    public static final double DRIVE_VINTERCEPT = 1.055; // V
    
    // Vision Constants
    public static final double VISION_STEER_kP = 0;
    public static final double VISION_STEER_kI = 0;
    public static final double VISION_TARGET_DEADBAND = 0.1; // degrees

    public static final double CAMERA_HEIGHTFROMGROUND = Units.inchesToMeters(24); // meters
    public static final double CAMERA_ANGLEFROMPARALLEL = Units.degreesToRadians(15); // radians
    public static final double TARGET_HEIGHTFROMGROUND = Units.inchesToMeters(94.75); // meters (7 ft 10.75 in, direct center of trapezoid)
    
    // public static final double CAMERAXOFFSET = 0;
    // public static final double CAMERAYOFFSET = 0;
    // public static final double CAMERAYAWANGLEDEGREES = 0;
}
