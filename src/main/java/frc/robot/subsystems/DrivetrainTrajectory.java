/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.lib.geometry.Twist;
import frc.lib.util.DriveSignal;
import frc.robot.constants.RobotConstants;
import frc.robot.helper.HDriveHelper;

public class DrivetrainTrajectory extends Drivetrain {
	
	private TalonFX leftMaster, rightMaster, leftFollower, rightFollower, normalMaster;
	private TalonSRX gyroHost;
	private PigeonIMU gyro;
	
	public DriveControlState DriveControlState = Drivetrain.DriveControlState.OPEN_LOOP;
	
	public DrivetrainTrajectory() {
		leftMaster = new TalonFX(RobotConstants.LEFT_MASTER_ID);
		leftMaster.configFactoryDefault();
		leftMaster.setInverted(false);
		leftMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		leftMaster.config_kP(0, RobotConstants.LEFT_kP);
		leftMaster.config_kI(0, RobotConstants.LEFT_kI);
		leftMaster.config_kD(0, RobotConstants.LEFT_kD);
		leftMaster.config_kF(0, RobotConstants.LEFT_kF);
		
		leftFollower = new TalonFX(RobotConstants.LEFT_SLAVE_ID);
		leftFollower.configFactoryDefault();
		leftFollower.follow(leftMaster);
		leftFollower.setInverted(false);
		leftFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		
		rightMaster = new TalonFX(RobotConstants.RIGHT_MASTER_ID);
		rightMaster.configFactoryDefault();
		rightMaster.setInverted(true);
		rightMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		rightMaster.config_kP(0, RobotConstants.RIGHT_kP);
		rightMaster.config_kI(0, RobotConstants.RIGHT_kI);
		rightMaster.config_kD(0, RobotConstants.RIGHT_kD);
		rightMaster.config_kF(0, RobotConstants.RIGHT_kF);
		
		rightFollower = new TalonFX(RobotConstants.RIGHT_SLAVE_ID);
		rightFollower.configFactoryDefault();
		rightFollower.follow(rightMaster);
		rightFollower.setInverted(true);
		rightFollower.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		
		normalMaster = new TalonFX(RobotConstants.NORMAL_ID);
		normalMaster.configFactoryDefault();
		normalMaster.setInverted(false);
		normalMaster.configStatorCurrentLimit(RobotConstants.TALON_CURRENT_LIMIT);
		normalMaster.config_kP(0, RobotConstants.NORMAL_kP);
		normalMaster.config_kI(0, RobotConstants.NORMAL_kI);
		normalMaster.config_kD(0, RobotConstants.NORMAL_kD);
		normalMaster.config_kF(0, RobotConstants.NORMAL_kF);
		
		gyroHost = new TalonSRX(6);
		gyroHost.configFactoryDefault();
		gyro = new PigeonIMU(gyroHost);
	}
	
	@Override
	public void drive(double leftPercent, double rightPercent, double normalPercent) {
		leftMaster.set(ControlMode.PercentOutput, leftPercent);
		rightMaster.set(ControlMode.PercentOutput, rightPercent);
		normalMaster.set(ControlMode.PercentOutput, normalPercent);
	}
	
	@Override
	public void drive(Twist velocity) {
		drive(HDriveHelper.hDrive(velocity.getTranslation(), velocity.getRotation()));
	}
	
	@Override
	public void drive(DriveSignal driveSignal) {
		drive(driveSignal, ControlMode.Velocity);
	}
	
	public void drive(DriveSignal driveSignal, ControlMode controlMode) {
		// System.out.println(driveSignal);
		leftMaster.set(ControlMode.Velocity, driveSignal.getLeft());
		rightMaster.set(ControlMode.Velocity, driveSignal.getRight());
		normalMaster.set(ControlMode.Velocity, driveSignal.getNormal());
	}
	
	@Override
	public void followPath() {
		
	}
	
	@Override
	public void reset() {
		zeroSensors();
	}
	
	@Override
	public void zeroSensors() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);
		gyro.setFusedHeading(0);
	}
	
	public int getLeftEncoderPosition() {
		return (int) leftMaster.getSelectedSensorPosition();
	}
	
	public int getRightEncoderPosition() {
		return (int) rightMaster.getSelectedSensorPosition();
	}
	
	public double getGyroAngle() {
		return gyro.getFusedHeading();
	}
}
