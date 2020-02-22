/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTargetCommand extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Vision vision;
	private final Drivetrain drivetrain;
	
	private double degreesRotate = 10;
	private double inDeadbandTime = 0;
	
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
		degreesRotate = 10;
		inDeadbandTime = 0;
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(vision.hasValidTarget()) {
			degreesRotate = vision.getLimelight().getdegRotationToTarget();
            double steer_cmd = RobotConstants.VISION_kSTEER * degreesRotate;
			drivetrain.drive(0, 0, steer_cmd);
			if(Util.deadband(degreesRotate, RobotConstants.VISION_TARGET_DEADBAND) == 0) {
				inDeadbandTime = Timer.getFPGATimestamp();
			} else {
				inDeadbandTime = -1;
			}
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
		//		if greater than 0 and	80 milliseconds have passed		and		we are still within deadband
		return (inDeadbandTime > 0) && (inDeadbandTime + 0.08 < Timer.getFPGATimestamp()) &&
		 Util.deadband(degreesRotate, RobotConstants.VISION_TARGET_DEADBAND) == 0;
	}
}
