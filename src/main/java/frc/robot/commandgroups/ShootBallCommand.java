package frc.robot.commandgroups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.Util;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class ShootBallCommand extends SequentialCommandGroup {
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public ShootBallCommand(Intake intake, Shooter shooter, double conveyerTime, boolean inverted) {
		super(
			new PrintCommand("Shoot Ball"),
			new ParallelDeadlineGroup(
				new WaitCommand(0.5), 
				new MoveConveyorCommand(intake, inverted),
				new IntakeBallCommand(intake, 0.5, inverted)),
			new ParallelDeadlineGroup(
				new WaitUntilCommand(() -> shooter.atSpeed(RobotConstants.FLYWHEEL_RPM_DEADBAND)),
				new MoveConveyorCommand(intake, inverted),
				new IntakeBallCommand(intake, 0.5, inverted)
			),
			// new ParallelDeadlineGroup(
			// 	new WaitCommand(conveyerTime), 
			// 	new MoveConveyorCommand(intake, inverted),
			// 	new IntakeBallCommand(intake, 0.5, inverted)
			// ),
			new ParallelDeadlineGroup(
				new FeedShooterCommand(intake, shooter, inverted).withTimeout(0.42069),
				new MoveConveyorCommand(intake, inverted),
				new IntakeBallCommand(intake, 0.5, inverted)
			)
		);
	}
}