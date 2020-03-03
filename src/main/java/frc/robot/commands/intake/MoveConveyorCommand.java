/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
* An example command that uses an example subsystem.
*/
public class MoveConveyorCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Intake intake;
	private final double speed;
	private final boolean inverted;
	
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public MoveConveyorCommand(Intake intake, double speed, boolean inverted) {
		this.intake = intake;
		this.speed = speed;
		this.inverted = inverted;
		// Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(intake);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		System.out.println("CONVEYER MOVE");
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intake.rollConveyor(speed, inverted);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.rollConveyor(0, false);
		System.out.println("CONVEYER MOVE COMMAND | END");
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
