/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
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
		double forward = RobotContainer.subsystemController.rightStick.getYCubedWithDeadband(0.07);
		double normal = RobotContainer.subsystemController.rightStick.getXCubedWithDeadband(0.07);
		double turn = RobotContainer.subsystemController.leftStick.getXCubedWithDeadband(0.07);
		
		if(drivetrain.getDriveMode() == DriveMode.FIELD_CENTRIC) {
			drivetrain.driveFieldCentric(forward, normal, turn, drivetrain.getGyroAngle());
		} else {
			drivetrain.drive(forward, normal, turn);
		}
		
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle());
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
