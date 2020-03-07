package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveFollowTrajectory;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeArm;
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
    public SixBallAutoCommand(Vision vision, Drivetrain drivetrain, Feeder feeder, Conveyer conveyer, IntakeArm intakeArm, Shooter shooter, Banana banana) {
        addCommands(
            new TurnAndShootFullAutoCommand(vision, drivetrain, feeder, conveyer, intakeArm , shooter, banana),
            // new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_DEPLOYED),
            new ParallelDeadlineGroup(
                new DriveFollowTrajectory(drivetrain, TrajectoryGenerator.leftTrajectory, TrajectoryGenerator.rightTrajectory),
                new StopFlywheel(shooter),
                new IntakeBallCommand(intakeArm, 0.5, false) // never ends
            ),
            new TurnAndShootFullAutoCommand(vision, drivetrain, feeder, conveyer, intakeArm , shooter, banana)
        );
    }

}