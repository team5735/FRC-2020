/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Intake extends SubsystemBase {
	
	private final TalonSRX intakeArm;
	private final VictorSPX intakeRoller;
	private final TalonSRX conveyorRoller;
	private final VictorSPX conveyorFeeder;

	private final DigitalInput retractedLimitSwitch, deployedLimitSwitch;
	
	/**
	* Creates a new Intake.
	*/
	public Intake() {    
		intakeArm = new TalonSRX(7);
		intakeArm.configFactoryDefault();
		intakeArm.config_kP(0, RobotConstants.INTAKE_kP);
		intakeArm.config_kI(0, RobotConstants.INTAKE_kI);
		intakeArm.config_kD(0, RobotConstants.INTAKE_kD);

		intakeRoller = new VictorSPX(10);
		intakeRoller.configFactoryDefault();
		intakeArm.overrideLimitSwitchesEnable(true);

		conveyorRoller = Drivetrain.gyroHost; // shared TalonSRX

		conveyorFeeder = new VictorSPX(4);
		conveyorFeeder.configFactoryDefault();
		
		retractedLimitSwitch = new DigitalInput(0);
		deployedLimitSwitch = new DigitalInput(1);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * @return Position, in sensor units
	 */
	public double getPosition() {
		return intakeArm.getSelectedSensorPosition();
	}

	public void resetPosition() {
		intakeArm.setSelectedSensorPosition(0);
	}

	public void moveArm(ControlMode mode, double value) {
		intakeArm.set(mode, value);
	}
	
	public void intakeBall(double speed, boolean inverted) {
		intakeRoller.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * speed);
	}

	public void rollConveyor(double speed, boolean inverted) {
		conveyorRoller.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * (speed < 0 ? 0.2 : speed));
	}

	public void feedShooter(double speed) {
		conveyorFeeder.set(ControlMode.PercentOutput, (speed < 0 ? 0.05 : speed));
	}

	public boolean isRetractedLimitHit() {
		return retractedLimitSwitch.get();
	}

	public boolean isDeployedLimitHit() {
		return deployedLimitSwitch.get();	
	}
}
