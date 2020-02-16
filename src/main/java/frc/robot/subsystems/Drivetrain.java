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

  public abstract void drive(double leftPercent, double rightPercent, double normalPercent);

  public abstract void drive(Twist velocity);

  public abstract void drive(DriveSignal driveSignal);

  public abstract void followPath();

  public abstract void reset();

  public abstract void zeroSensors();
}
