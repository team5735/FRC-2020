package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TurnAndShootCommand extends SequentialCommandGroup {

    private Shooter shooter;

    /**
     * Turn to target, ramp up shooter, feed the shooter, and go!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public TurnAndShootCommand(Vision vision, Drivetrain drivetrain, Intake intake, Shooter shooter) {
        this.shooter = shooter;
        addCommands(
            // https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html
            new ParallelRaceGroup(
                new TurnToTargetCommand(vision, drivetrain),
                new RampShooterCommand(shooter, RobotConstants.FLYWHEEL_PRESET_TRENCH)//shooter.getSpeedFromDistance(vision.getDistanceToTarget())),    
            ),
            new FeedShooterCommand(intake, false).withTimeout(3),
            new StopFlywheel(shooter)
        );
    }

    // @Override
    // public void end(boolean interrupted) {
    //     // TODO Auto-generated method stub
    //     super.end(interrupted);
    //     CommandScheduler.getInstance().schedule(new RampShooterCommand(shooter, 0));
    // }
    
}