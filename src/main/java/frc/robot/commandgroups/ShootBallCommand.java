/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.subsystems.Intake;

/**
* An example command that uses an example subsystem.
*/
public class ShootBallCommand extends ParallelCommandGroup{
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public ShootBallCommand(Intake intake, boolean inverted) {
		addCommands(
			new MoveConveyorCommand(intake, inverted),
			new IntakeBallCommand(intake, 0.5, inverted),
			new FeedShooterCommand(intake, inverted).withTimeout(0.09)
		);
	}
}
