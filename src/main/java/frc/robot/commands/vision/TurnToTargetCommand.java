/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTargetCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Vision vision;
    private final Drivetrain drivetrain;
	
	public TurnToTargetCommand(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
		
		// Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
        addRequirements(drivetrain);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        vision.enableTracking();
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(vision.hasValidTarget()) {
            double steer_cmd = RobotConstants.VISION_kSTEER * vision.getLimelight().getdegRotationToTarget();
            drivetrain.drive(0, 0, steer_cmd);
        }
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		vision.disableTracking();
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return Util.deadband(vision.getLimelight().getdegRotationToTarget(), RobotConstants.VISION_TARGET_DEADBAND) == 0;
	}
}
