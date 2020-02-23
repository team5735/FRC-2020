/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Banana extends SubsystemBase {

  public static final double gearRatio = 669696969;

  private final TalonSRX banana;

  /**
   * Creates a new Banana.
   */
  public Banana() {
    banana = new TalonSRX(RobotConstants.BANANA_ID);
    banana.configFactoryDefault();
    banana.config_kP(0, 0.01);
    banana.config_kI(0, 0);
    banana.config_kD(0, 0);
    banana.config_kF(0, 0);
  }

  public double getPosition() {
		return banana.getSelectedSensorPosition();
	}

	public void resetPosition() {
		banana.setSelectedSensorPosition(0);
	}

	public void moveBanana(ControlMode mode, double value) {
		banana.set(mode, value);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
