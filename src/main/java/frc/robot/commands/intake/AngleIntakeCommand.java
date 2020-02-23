/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Intake;

/**
* An example command that uses an example subsystem.
*/
public class AngleIntakeCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Intake intake;
	private final double position;
	private final boolean inverted;
	
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public AngleIntakeCommand(Intake intake, double position) {
		this.intake = intake;
		this.position = position;
		this.inverted = false;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intake);
	}

	public AngleIntakeCommand(Intake intake, boolean inverted) {
		this.intake = intake;
		this.position = -1;
		this.inverted = inverted;
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// intake.moveArm(ControlMode.Position, position);
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// System.out.println(intake.getPosition());
		// double position = intake.getPosition();
		// if(position < RobotConstants.INTAKE_POSITION_RETRACTED ||
		// 	position > RobotConstants.INTAKE_POSITION_DEPLOYED) end(true);
		intake.moveArm(ControlMode.PercentOutput, (inverted ? -1 : 1) * 0.2);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// intake.moveArm(ControlMode.Position, intake.getPosition()); // stay
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return Util.deadband(intake.getPosition(), RobotConstants.INTAKE_POSITION_DEADBAND) == 0;
		return false;
	}
}
