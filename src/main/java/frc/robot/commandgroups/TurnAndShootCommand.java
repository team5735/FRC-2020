package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TurnAndShootCommand extends SequentialCommandGroup {
    /**
     * Turn to target, ramp up shooter, feed the shooter, and go!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public TurnAndShootCommand(Vision vision, Drivetrain drivetrain, Intake intake, Shooter shooter) {
        addCommands(
            new TurnToTargetCommand(vision, drivetrain),
            new RampShooterCommand(shooter, shooter.getSpeedFromDistance(vision.getDistanceToTarget())),
            new FeedShooterCommand(intake)
        );
    }
    
}