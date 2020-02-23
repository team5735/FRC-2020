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
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Intake;

/**
* An example command that uses an example subsystem.
*/
public class ZeroIntakeCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Intake intake;
	
	/**
	* Creates a new ExampleCommand.
	*
	* @param subsystem The subsystem used by this command.
	*/
	public ZeroIntakeCommand(Intake intake) {
		this.intake = intake;
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
        intake.moveArm(ControlMode.PercentOutput, -0.1);
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        intake.moveArm(ControlMode.PercentOutput, 0);
        intake.resetPosition();
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
		// return intake.isRetractedLimitHit();
	}
}
