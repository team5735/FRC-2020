/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSpinner extends SubsystemBase {
  private TalonSRX talon;
  private double ratio, deltaCompensation;

  private static final int OFFSET = 2;
  private static final int DRIVE_DIAMETER = 4;

  /**
   * Creates a new ExampleSubsystem.
   */
  public ColorSpinner() {
    talon = new TalonSRX(1);
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    talon.setInverted(false);
    talon.configContinuousCurrentLimit(39);
    talon.enableCurrentLimit(true);
    talon.configMotionCruiseVelocity(4095);
    talon.configMotionAcceleration(1023);

    talon.selectProfileSlot(0, 0);
    talon.config_kP(0, 1);
    talon.config_kI(0, 0);
    talon.config_kD(0, 10);
    talon.config_kF(0, 0.5);

    ratio = 32.0 / DRIVE_DIAMETER * 4.0;
    deltaCompensation = 1.075;

    SmartDashboard.putNumber("Revolutions", 1);
  }

  public void init() {
    talon.setSelectedSensorPosition(0);
  }

  public void spin(double revolutions) {
    talon.set(ControlMode.MotionMagic, talon.getSelectedSensorPosition() + revolutionsToEncoderTicks(revolutions));
  }

  public void stop() {
    talon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double revolutionsToEncoderTicks(double revolutions){
    return revolutions * 1024.0 * ratio * deltaCompensation;
  }

}
