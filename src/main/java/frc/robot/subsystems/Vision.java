/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimeLight;
import frc.lib.limelight.LimelightControlMode.CamMode;
import frc.lib.limelight.LimelightControlMode.LedMode;
import frc.robot.constants.RobotConstants;

public class Vision extends SubsystemBase {

	private LimeLight limelight;
	private boolean trackingMode = false;
	private boolean hasValidTarget = false;

	public enum LimelightPipeline {
		TESTING(0), NORTH_SHORE(1), GREATER_BOSTON(2);

		private final int value;
		private LimelightPipeline(int id) {
			this.value = id;
		}

		public int getValue() {
			return value;
		}
	};
	
	public Vision() {
		limelight = new LimeLight();
		limelight.setPipeline(LimelightPipeline.TESTING.getValue());
		limelight.setCamMode(CamMode.kdriver);
		limelight.setLEDMode(LedMode.kforceOff);

		CommandScheduler.getInstance().registerSubsystem(this);
	}
	
	@Override
	public void periodic() {
		if(trackingMode) hasValidTarget = limelight.getIsTargetFound();
	}

	/**
	 * @return Horizontal distance to target, in meters
	 */
	public double getDistanceToTarget() {
		if(!trackingMode) enableTracking();
		boolean gotDistance = false;
		
		double heightDiff = RobotConstants.TARGET_HEIGHTFROMGROUND - RobotConstants.CAMERA_HEIGHTFROMGROUND;
		double distance = 0;

		while(!gotDistance) {
			double yAngleToTarget = Units.degreesToRadians(limelight.getdegVerticalToTarget()); // radians
			distance = heightDiff / Math.tan(RobotConstants.CAMERA_ANGLEFROMPARALLEL + yAngleToTarget); // meters
			if(distance > 1) {
				gotDistance = true;
			}
		}

		disableTracking();

		// SmartDashboard.putNumber("Distance to Target (m)", distance);

		return distance;
	}

	public void enableTracking() {
		limelight.setCamMode(CamMode.kvision);
		limelight.setLEDMode(LedMode.kforceOn);
		trackingMode = true;
	}

	public void disableTracking() {
		limelight.setCamMode(CamMode.kdriver);
		limelight.setLEDMode(LedMode.kforceOff);
		trackingMode = false;
	}

	public boolean hasValidTarget() {
		return hasValidTarget;
	}

	public LimeLight getLimelight() {
		return limelight;
	}
}
