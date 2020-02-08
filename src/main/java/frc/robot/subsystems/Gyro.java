/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

/**
 * Add your docs here.
 */
public class Gyro extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private PigeonIMU gyro;
  protected double[] ypr = new double[3];

  public Gyro() {
    gyro = new PigeonIMU(RobotConstants.GYRO_TALON_HOST_ID);
  }

  public void getIMUYPR() {
    gyro.getYawPitchRoll(ypr);
  }
}
