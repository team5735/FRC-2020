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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.constants.RobotConstants;

public class Intake extends SubsystemBase {
	
	private final TalonSRX intakeArm;
	private final VictorSPX intakeRoller;
	private final VictorSPX conveyorRoller;
	private final TalonSRX conveyorFeeder;

	// private final DigitalInput retractedLimitSwitch, deployedLimitSwitch;
	
	/**
	* Creates a new Intake.
	*/
	public Intake() {    
		intakeArm = new TalonSRX(RobotConstants.INTAKE_ARM_ID);
		intakeArm.configFactoryDefault();
		intakeArm.config_kP(0, RobotConstants.INTAKE_kP);
		intakeArm.config_kI(0, RobotConstants.INTAKE_kI);
		intakeArm.config_kD(0, RobotConstants.INTAKE_kD);
		intakeArm.overrideLimitSwitchesEnable(true);
		resetPosition();

		intakeRoller = new VictorSPX(RobotConstants.INTAKE_ROLLER_ID);
		intakeRoller.configFactoryDefault();
		intakeRoller.setInverted(true);

		conveyorFeeder = Drivetrain.gyroHost; // shared TalonSRX

		conveyorRoller = new VictorSPX(RobotConstants.CONVEYOR_ID);
		conveyorRoller.configFactoryDefault();
		conveyorRoller.setInverted(true);
		
		// retractedLimitSwitch = new DigitalInput(9);
		// deployedLimitSwitch = new DigitalInput(1);

		// CommandScheduler.getInstance().setDefaultCommand(this, new IntakeBallCommand(this));
		// CommandScheduler.getInstance().setDefaultCommand(this, new AngleIntakeCommand(this, 0));

	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Intake Angle Sensor Value", getPosition());

		// if(isDeployedLimitHit() || isRetractedLimitHit()) {
			// intakeArm.set(ControlMode.PercentOutput, 0);
			// moveArm(ControlMode.Position, getPosition());
		// }
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
		conveyorRoller.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * speed);
	}

	public void feedShooter(double speed, boolean inverted) {
		conveyorFeeder.set(ControlMode.PercentOutput, (inverted ? -1 : 1) * (speed < 0 ? 0.3 : speed));
	}

	// public boolean isRetractedLimitHit() {
	// 	return retractedLimitSwitch.get();
	// }

	// public boolean isDeployedLimitHit() {
	// 	return deployedLimitSwitch.get();	
	// }
}
