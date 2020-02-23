/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RampShooterCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Shooter shooter;
	private final double rpm;
	
	public RampShooterCommand(Shooter shooter, double rpm) {
		this.shooter = shooter;
		this.rpm = rpm;
		
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.shooter);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if(rpm == 0) {
			shooter.slowDown();
		} else {
			shooter.setSpeed(rpm); // only need to call once?
		}
		
		// TODO: Adjust banana as necessary
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// SmartDashboard.putNumber("Shooter Speed (RPM)", shooter.getSpeed());
		double speed = shooter.getSpeed();
		System.out.println("Shooter Speed (RPM): " + speed);
		System.out.println("Error: " + (speed - RobotConstants.FLYWHEEL_PRESET_LINE));
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("RAMP SHOOTER COMMAND | END");
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.deadband(shooter.getSpeed() - rpm, 40) == 0;
		// return false;
	}
}
