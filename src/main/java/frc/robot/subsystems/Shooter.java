/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.InterpolatingDouble;

import java.util.Map;

public class Shooter extends SubsystemBase {

	private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> DistanceToRPM;

	private CANSparkMax neoMaster, neoSlave;
	private CANPIDController flywheelPIDController;

	private double speedSetpoint;

	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {

		neoMaster = new CANSparkMax(RobotConstants.FLYWHEEL_MASTER_ID, MotorType.kBrushless);
		neoMaster.restoreFactoryDefaults();
		neoMaster.setInverted(false);
		neoMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);


		neoSlave = new CANSparkMax(RobotConstants.FLYWHEEL_SLAVE_ID, MotorType.kBrushless);
		neoSlave.restoreFactoryDefaults();
		neoSlave.follow(neoMaster, true);
		neoSlave.enableSoftLimit(SoftLimitDirection.kReverse, true);

		neoMaster.getEncoder().setVelocityConversionFactor(RobotConstants.FLYWHEEL_PULLEY_RATIO);

		flywheelPIDController = neoMaster.getPIDController();

		flywheelPIDController.setP(RobotConstants.FLYWHEEL_kP, 0);
		flywheelPIDController.setI(RobotConstants.FLYWHEEL_kI, 0);
		flywheelPIDController.setD(RobotConstants.FLYWHEEL_kD, 0);
		flywheelPIDController.setFF(RobotConstants.FLYWHEEL_kF, 0);

		DistanceToRPM = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(1000);
		// DistanceToRPM.put(new InterpolatingDouble(RobotConstants.DISTANCE_TO_TARGET_PRESET_LINE),
		// 		new InterpolatingDouble(RobotConstants.FLYWHEEL_PRESET_LINE));
		// DistanceToRPM.put(new InterpolatingDouble(RobotConstants.DISTANCE_TO_TARGET_PRESET_TRENCH),
		// 		new InterpolatingDouble(RobotConstants.FLYWHEEL_PRESET_TRENCH));
		// DistanceToRPM.put(new InterpolatingDouble(RobotConstants.DISTANCE_TO_TARGET_PRESET_BEHINDCOLORWHEEL),
		// 		new InterpolatingDouble(RobotConstants.FLYWHEEL_PRESET_BEHINDCOLORWHEEL));
		DistanceToRPM.put(new InterpolatingDouble(3.75476), new InterpolatingDouble(3500.0));
		DistanceToRPM.put(new InterpolatingDouble(4.4367), new InterpolatingDouble(3600.0));
		DistanceToRPM.put(new InterpolatingDouble(4.9945), new InterpolatingDouble(3650.0));
		DistanceToRPM.put(new InterpolatingDouble(5.5089), new InterpolatingDouble(3850.0));
		// DistanceToRPM.put(new InterpolatingDouble(3.75476), new InterpolatingDouble(3500.0));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setSpeed(double rpm) {
		double speed = rpm;
		if (rpm > RobotConstants.FLYWHEEL_MAX_SPEED)
			speed = RobotConstants.FLYWHEEL_MAX_SPEED;
		if (rpm < RobotConstants.FLYWHEEL_MIN_SPEED)
			speed = RobotConstants.FLYWHEEL_MIN_SPEED;
		speedSetpoint = speed;
		flywheelPIDController.setReference(speed, ControlType.kVelocity);
	}

	public double getSpeed() {
		return neoMaster.getEncoder().getVelocity();
	}

	public boolean atSpeed(double threshold) {
		System.out.println("Shooter Speed (RPM): " + getSpeed());
		return Util.deadband(speedSetpoint - getSpeed(), threshold) == 0;
	}

	public void slowDown() {
		neoMaster.set(0);
		speedSetpoint = 0;
	}

	/**
	 * Function to convert a distance into flywheel speed
	 * 
	 * @param distance Horizontal distance to target, in meters
	 * @return Flywheel speed, in RPM
	 */
	public double getSpeedFromDistance(double distance) {
		return DistanceToRPM.getInterpolated(new InterpolatingDouble(distance)).value;
	}

}
