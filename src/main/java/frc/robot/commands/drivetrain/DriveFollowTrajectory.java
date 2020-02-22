/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
* An example command that uses an example subsystem.
*/
public class DriveFollowTrajectory extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Drivetrain s_drivetrain;
	private Trajectory leftTraj, rightTraj;
	private EncoderFollower left, right;
	private boolean isTrajDone = false;
	
	/**
	* Creates a new FollowTrajectory.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public DriveFollowTrajectory(Drivetrain subsystem, Trajectory leftTraj, Trajectory rightTraj) {
		s_drivetrain = subsystem;
		this.leftTraj = leftTraj;
		this.rightTraj = rightTraj;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements((Subsystem) subsystem);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("========== INITIALIZING ==========");
		
		s_drivetrain.reset();
		
		left = new EncoderFollower(leftTraj);
		left.configureEncoder(s_drivetrain.getLeftSidePosition(), (int) RobotConstants.ENCODER_TICKS_PER_WHEEL_REV, RobotConstants.WHEEL_DIAMETER);
		left.configurePIDVA(RobotConstants.AUTO_LEFT_kP, RobotConstants.AUTO_LEFT_kI, RobotConstants.AUTO_LEFT_kD, 1 / RobotConstants.MAX_VELOCITY, 0);
		
		right = new EncoderFollower(rightTraj);
		right.configureEncoder(s_drivetrain.getRightSidePosition(), (int) RobotConstants.ENCODER_TICKS_PER_WHEEL_REV, RobotConstants.WHEEL_DIAMETER);
		right.configurePIDVA(RobotConstants.AUTO_RIGHT_kP, RobotConstants.AUTO_RIGHT_kI, RobotConstants.AUTO_RIGHT_kD, 1 / RobotConstants.MAX_VELOCITY, 0);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(isTrajDone) return;
		int leftPos = s_drivetrain.getLeftSidePosition();
		int rightPos = s_drivetrain.getRightSidePosition();
		System.out.println("Left E: " + leftPos + ", Right E: " + rightPos);
		// RETURNS in PERCENT OUTPUT
		double l = left.calculate(leftPos);
		double r = right.calculate(rightPos);
		
		double gyro_heading = s_drivetrain.getGyroAngle();    // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
		// System.out.println("Heading: " + gyro_heading + " | Wanted: " + desired_heading);
		
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
		
		s_drivetrain.drivePercentOutput(l + turn, r - turn, 0);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("DONE: L current:" + Units.rotationsToMeters(Units.tickstoRotations(s_drivetrain.getLeftSidePosition())) / 6.2222 + ", R current: " + Units.rotationsToMeters(Units.tickstoRotations(s_drivetrain.getRightSidePosition())) / 6.2222);
		s_drivetrain.drive(0, 0, 0);
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(left.isFinished() && right.isFinished()) {
			isTrajDone = true;
			return true;
		}
		return false;
	}
}
