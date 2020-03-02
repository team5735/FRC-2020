package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.Vision;

public class TurnAndPrepareCommand extends SequentialCommandGroup {
    /**
     * Get balls, shoot, get more balls, shoot!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public TurnAndPrepareCommand(Vision vision, Drivetrain drivetrain, Intake intake, Shooter shooter, Banana banana) {
        addCommands(
            new ParallelCommandGroup(
                new TurnToTargetCommand(vision, drivetrain),
                new RampShooterCommand(shooter, vision, banana, 3650)
            ), 
            new RampShooterCommand(shooter, vision, banana, shooter.getSpeedFromDistance(vision.getDistanceToTarget()))
        );
    }

}