/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Intake;

/**
* An example command that uses an example subsystem.
*/
public class AngleIntakeCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Intake intake;
	private final double position;
	
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public AngleIntakeCommand(Intake intake, double position) {
		this.intake = intake;
		this.position = position;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intake);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intake.moveArm(position);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.moveArm(intake.getPosition()); // stay
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.deadband(intake.getPosition(), RobotConstants.INTAKE_POSITION_DEADBAND) == 0;
	}
}
