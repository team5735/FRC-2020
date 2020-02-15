/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Twist;
import frc.lib.util.DriveSignal;
import frc.robot.constants.RobotConstants;

/**
 * Add your docs here.
 */
public abstract class Drivetrain extends SubsystemBase {

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING // velocity PID control
  }

  public abstract void drive(Twist velocity);

  public abstract void drive(DriveSignal DriveSignal);

  public abstract void followPath();

  public abstract void reset();

  public abstract void zeroSensors();

  public abstract int getSensorPositionLeft();

  public abstract int getSensorPositionRight();

  public abstract int getSensorPositionNormal();


  public static double rotationsToInches(double rotations) {
    return rotations * (RobotConstants.DriveWheelDiameterInches * Math.PI);
  }

  public static double inchesToRotations(double inches) {
    return inches / (RobotConstants.DriveWheelDiameterInches * Math.PI);
  }

  public static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60.0;
  }

  public static double radiansPerSecondToTicksPer100ms(double radians) {
    return radians / (Math.PI * 2.0) * RobotConstants.EncoderTicksPerRotation / 10.0;
  }

  public static double rpmToTicksPer100ms(double rpm) {
    return rpm * RobotConstants.EncoderTicksPerRotation / 60.0 / 10.0;
  }

  public static double rotationstoTicks(double rotations) {
    return rotations * RobotConstants.EncoderTicksPerRotation;
  }

  public static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60.0;
  }
  
  public static double radiansPerSecondToRPM(double radians) {
    return radians / (Math.PI * 2.0) * 60.0;
  }
}
