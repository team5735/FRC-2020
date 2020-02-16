/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainTrajectory;
import frc.robot.subsystems.TrajectoryGenerator;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
* An example command that uses an example subsystem.
*/
public class DriveFollowTrajectory extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final DrivetrainTrajectory s_drivetrain;
	private EncoderFollower left, right;
	
	/**
	* Creates a new FollowTrajectory.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public DriveFollowTrajectory(DrivetrainTrajectory subsystem) {
		s_drivetrain = subsystem;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("========== INITIALIZING ==========");
		
		s_drivetrain.reset();
		
		left = new EncoderFollower(TrajectoryGenerator.leftTrajectory);
		left.configureEncoder(s_drivetrain.getLeftEncoderPosition(), (int) RobotConstants.ENCODER_TICKS_PER_REV, RobotConstants.WHEEL_DIAMETER);
		left.configurePIDVA(RobotConstants.LEFT_kP, RobotConstants.LEFT_kI, RobotConstants.LEFT_kD, 1 / RobotConstants.MAX_VELOCITY, 0);
		
		right = new EncoderFollower(TrajectoryGenerator.rightTrajectory);
		right.configureEncoder(s_drivetrain.getRightEncoderPosition(), (int) RobotConstants.ENCODER_TICKS_PER_REV, RobotConstants.WHEEL_DIAMETER);
		right.configurePIDVA(RobotConstants.RIGHT_kP, RobotConstants.RIGHT_kI, RobotConstants.RIGHT_kD, 1 / RobotConstants.MAX_VELOCITY, 0);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		int leftEncoder = s_drivetrain.getLeftEncoderPosition();
		int rightEncoder = s_drivetrain.getRightEncoderPosition();
		System.out.println("Left E: " + leftEncoder + ", Right E: " + rightEncoder);
		// RETURNS in PERCENT OUTPUT
		double l = left.calculate(leftEncoder);
		double r = right.calculate(rightEncoder);
		
		double gyro_heading = s_drivetrain.getGyroAngle();    // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
		System.out.println("Heading: " + gyro_heading + " | Wanted: " + desired_heading);
		
		// This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		angleDifference = angleDifference % 360.0;
		if (Math.abs(angleDifference) > 180.0) {
			angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
		} 
		// if(angleDifference > 185) angleDifference = 0;
		
		double turn = RobotConstants.kTURN_CORRECTION * angleDifference;
		// double turn = 0;
		
		System.out.println("@@@@@@@@@@@@@@@ Left: " + (l + turn) + ", Right: " + (r - turn) + ", Angle Diff: " + angleDifference + ", Turn: " + turn);
		
		s_drivetrain.drive(l + turn, r - turn, 0);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return left.isFinished() && right.isFinished();
	}
}
