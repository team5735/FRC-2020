package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.Vision;

public class SixBallAutoCommand extends SequentialCommandGroup {
    /**
     * Get balls, shoot, get more balls, shoot!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public SixBallAutoCommand(Vision vision, Drivetrain drivetrain, Intake intake, Shooter shooter) {
        addCommands(
            new RampShooterCommand(shooter, shooter.getSpeedFromDistance(vision.getDistanceToTarget())),
            new FeedShooterCommand(intake).withTimeout(3),
            new RampShooterCommand(shooter, 0),
            new DriveAndSuccCommand(drivetrain, intake, TrajectoryGenerator.leftTrajectory, TrajectoryGenerator.rightTrajectory),
            new TurnToTargetCommand(vision, drivetrain),
            new RampShooterCommand(shooter, shooter.getSpeedFromDistance(vision.getDistanceToTarget())),
            new FeedShooterCommand(intake).withTimeout(3)
        );
    }
    
}