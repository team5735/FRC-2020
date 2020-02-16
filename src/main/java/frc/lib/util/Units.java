package frc.lib.util;

import frc.robot.constants.RobotConstants;

public class Units {
    
    /* Distance Conversion */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public static double feetToMeters(double feet) {
        return inchesToMeters(feet * 12.0);
    }

    public static double metersToFeet(double meters) {
        return metersToInches(meters) / 12.0;
    }

    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    /* Sensor Conversion */
    public static double rotationsToMeters(double rotations) {
        return rotations * (RobotConstants.WHEEL_DIAMETER * Math.PI);
    }

    public static double metersToRotations(double inches) {
        return inches / (RobotConstants.WHEEL_DIAMETER * Math.PI);
    }

    public static double rpmToTicks(double rpm) {
        return rpm * RobotConstants.ENCODER_TICKS_PER_REV / 60.0 / 10.0;
    }

    public static double metersPerSecondToRpm(double metersPerSec) {
        return metersToRotations(metersPerSec) * 60.0;
    }
    
    public static double radiansPerSecondToRPM(double radians) {
        return radians / (Math.PI * 2.0) * 60.0;
    }

    public static double radiansPerSecondToTicks(double radians) {
        return radians / (Math.PI * 2.0) * RobotConstants.ENCODER_TICKS_PER_REV / 10.0;
    }

}