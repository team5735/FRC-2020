/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.geometry.Pose;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Twist;
import frc.lib.physics.DCMotorTransmission;
import frc.lib.physics.DifferentialDrive;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.lib.trajectory.DistanceView;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.TrajectorySamplePoint;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.lib.trajectory.timing.TimingUtil;
import frc.robot.constants.RobotConstants;
import frc.robot.helper.HDriveHelper;
import frc.robot.Robot;
import frc.robot.commands.DriveJoystick;

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

  protected static double rotationsToInches(double rotations) {
    return rotations * (RobotConstants.DriveWheelDiameterInches * Math.PI);
  }

  protected static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }

  protected static double radiansPerSecondToTickesPer100ms(double radians) {
    return radians / (Math.PI * 2.0) * 4096.0 / 10.0;
  }

  protected static double inchesToRotations(double inches) {
    return inches / (RobotConstants.DriveWheelDiameterInches * Math.PI);
  }

  protected static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
  }

}
