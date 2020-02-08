/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.TrajectoryGenerator.TrajectorySet;

public class DriveFollowTrajectory extends CommandBase {
	private final Drivetrain drivetrain;
	private final TrajectoryGenerator generator;
	private final TrajectoryIterator<TimedState<PoseWithCurvature>> trajectory;
	
	/**
	* Creates a new DriveFollowTrajectory.
	*/
	public DriveFollowTrajectory(Trajectory<TimedState<PoseWithCurvature>> trajectory, Drivetrain drivetrain, TrajectoryGenerator generator) {
		this.drivetrain = drivetrain;
		this.generator = generator;
		this.trajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
		addRequirements(generator);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// TrajectorySet trajectorySet = generator.getTrajectorySet();
		if(generator.getTrajectorySet() != null) {
			generator.generateTrajectories();
		}
		Robot.robotState.reset(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
		drivetrain.setTrajectory(trajectory);

	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println("following");
		drivetrain.followPath();
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println(generator.getTrajectorySet().sideStartToNearScale.left.toString());
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return drivetrain.isDoneWithTrajectory();
	}
}
