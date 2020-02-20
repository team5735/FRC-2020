/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.geometry.Twist;
import frc.lib.util.Units;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class DriveJoystick extends CommandBase {
	
	private Drivetrain drivetrain;
	
	private double TURN_CONSTANT = 0.33;
	
	// private double maxV = 0;
	
	/**
	* Creates a new DriveJoystick.
	*/
	public DriveJoystick(Drivetrain drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.drivetrain = drivetrain;
		
		addRequirements((Subsystem) drivetrain);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		// drivetrain.drive(-RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07), -RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07), 0);
		// double velocity = ((DrivetrainTrajectory) drivetrain).getVelocity();
		// if(velocity > maxV) maxV = velocity;
		// SmartDashboard.putNumber("Max Velocity", maxV);
		// maxV * 0.0003711243842 = max velocity m/s
		
		// double forward = maxVelocityInTicksper100ms * RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07);
		// SmartDashboard.putNumber("Wanted Velocity", forward);
		
		// drivetrain.drive(new Twist(0 * maxVelocityInTicksper100ms * RobotContainer.subsystemController.rightStick.getXCubedWithDeadband(0.07),
		// -1 * maxVelocityInTicksper100ms * RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07), 
		//  -1 * TURN_CONSTANT * RobotContainer.subsystemController.leftStick.getXCubedWithDeadband(0.07)));
		
		double forward = -1 * (1 - TURN_CONSTANT) * RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07);
		double normal = RobotContainer.subsystemController.rightStick.getXCubedWithDeadband(0.07);
		double turn = TURN_CONSTANT * RobotContainer.subsystemController.leftStick.getXCubedWithDeadband(0.07);
		
		if(drivetrain.getDriveMode() == DriveMode.FIELD_CENTRIC) {
			drivetrain.driveFC(forward, normal, turn, drivetrain.getGyroAngle());
		} else {
			drivetrain.drive(forward + turn, forward - turn, normal);
		}
		
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle());
		
		//  SmartDashboard.putNumber("Actual Velocity (L)", ((DrivetrainTrajectory) drivetrain).getLeftVelocity());
		// SmartDashboard.putNumber("Actual Velocity (R)", ((DrivetrainTrajectory) drivetrain).getRightVelocity());
		
		// System.out.println(unitgay);
		// drivetrain.drive(new DriveSignal(RobotContainer.subsystemController.getY(Hand.kLeft), 0.0, 0.0));
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
