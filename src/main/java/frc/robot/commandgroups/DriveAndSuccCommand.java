package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import jaci.pathfinder.Trajectory;

public class DriveAndSuccCommand extends ParallelRaceGroup {
    /**
     * Drive along a trajectory while intaking
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public DriveAndSuccCommand(Drivetrain drivetrain, Intake intake, Trajectory leftTraj, Trajectory rightTraj) {
        addCommands(
            new DriveFollowTrajectory(drivetrain, leftTraj, rightTraj),
            new IntakeBallCommand(intake, 0.25, false)
        );
    }
    
}