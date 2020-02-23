/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Banana;

public class MoveBananaCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	private final Banana banana;
	private final double angle; // radians
	
	public MoveBananaCommand(Banana banana, double angle) {
		this.banana = banana;
		this.angle = angle;
		
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(banana);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		banana.moveBanana(ControlMode.PercentOutput, -0.3*RobotContainer.subsystemController.leftStick.getYCubedWithDeadband(0.07));
		// SmartDashboard.putNumber("Shooter Speed (RPM)", shooter.getSpeed());
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// System.out.println("RAMP SHOOTER COMMAND | END");
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return Util.deadband(shooter.getSpeed() - rpm, 40) == 0;
		return false;
	}
}
