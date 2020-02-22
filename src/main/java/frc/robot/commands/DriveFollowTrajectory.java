/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.Pose;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainTrajectory;
import frc.robot.subsystems.TrajectoryGenerator;

public class DriveFollowTrajectory extends CommandBase {
	private final DrivetrainTrajectory drivetrain;
	private final TrajectoryGenerator trajectoryGenerator;
	private final Trajectory<TimedState<PoseWithCurvature>> trajectory;
	private TrajectoryIterator<TimedState<PoseWithCurvature>> path;

	/**
	 * Creates a new DriveFollowTrajectory.
	 */
	public DriveFollowTrajectory(Trajectory<TimedState<PoseWithCurvature>> trajectory,
			DrivetrainTrajectory drivetrain, TrajectoryGenerator trajectoryGenerator) {
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		this.trajectoryGenerator = trajectoryGenerator;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// TrajectorySet trajectorySet = generator.getTrajectorySet();
		System.out.println("BRUH");
		Robot.robotState.reset(Timer.getFPGATimestamp(), new Pose());
		path = null;
		drivetrain.resetCurrentTrajectory();
		trajectoryGenerator.generateTrajectories();
		path = new TrajectoryIterator<>(new TimedView<>(trajectory));
		drivetrain.setTrajectory(path);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		drivetrain.followPath();
		// System.out.println(Robot.robotState.getFieldToVehicle(Timer.getFPGATimestamp()));
		System.out.println(path.getState().toString());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("done");
		System.out.println(trajectory);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return drivetrain.isDone();
	}
}
