/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.DriveSignal;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class DriveJoystick extends CommandBase {
	
	private Drivetrain drivetrain;
	
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
		if(drivetrain.getDriveMode() == DriveMode.DISABLED) return;

		double forward = -RobotContainer.driverController.rightStick.getYCubedWithDeadband(0.07);
		double normal = RobotContainer.driverController.rightStick.getXCubedWithDeadband(0.2);
		double turn = RobotContainer.driverController.leftStick.getXCubedWithDeadband(0.07);

		// double leftPercentage = forwardVelocity * (1 - ANGULAR_PERCENTAGE) + angularVelocity * ANGULAR_PERCENTAGE;
		// double rightPercentage = forwardVelocity * (1 - ANGULAR_PERCENTAGE) - angularVelocity * ANGULAR_PERCENTAGE;
		// double sidewaysPercentage = sidewaysVelocity * (1 - ANGULAR_PERCENTAGE);
		
		// drive(controlMode, new DriveSignal(leftPercentage, rightPercentage, sidewaysPercentage));

		// DriveSignal controllerInput = new DriveSignal(forward)
		
		if(drivetrain.getDriveMode() == DriveMode.FIELD_CENTRIC) {
			drivetrain.driveFieldCentric(forward, normal, turn, drivetrain.getGyroAngle());
		} else {
			drivetrain.drivePercent(ControlMode.Velocity, forward, normal, turn, RobotConstants.MAX_VELOCITY_NORMAL_TICKS);
			// drivetrain.drive(ControlMode.Velocity, new DriveSignal(0, 0, normal * RobotConstants.MAX_VELOCITY_NORMAL_TICKS));
		}
		
		System.out.println("Gyro Angle: " + drivetrain.getGyroAngle());
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
