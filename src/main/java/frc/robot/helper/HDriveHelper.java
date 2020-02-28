/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helper;

import frc.lib.geometry.Rotation;
import frc.lib.geometry.Translation;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class HDriveHelper {

    public static DriveSignal xyLockedDrive(double x, double y) {
        return hDrive(x, y, 0);
    }

    // ticksper100ms
    public static DriveSignal hDrive(double x, double y, double radians) {
        double arcLength = rotation.radians() * RobotConstants.DRIVETRAIN_TRACK_WIDTH / 2.0; // in inches/100ms
        double left = translation.y()  - Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double right = translation.y() + Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double normal = translation.x();

        return new DriveSignal(left, right, normal);
    }

    public static DriveSignal correctField(Translation translation, Rotation rotation, double fieldAngle) {
        return hDrive(translation.rotateBy(Rotation.fromDegrees(fieldAngle)), rotation);
    }

    /* Uses poses

    // translation is direction of travel with magnitute
    // rotation is rotating 

    public static DriveSignal xyLockedDrive(Translation translation) {
        return hDrive(translation, Rotation.Identity);
    }

    // ticksper100ms
    public static DriveSignal hDrive(Translation translation, Rotation rotation) {
        double arcLength = rotation.radians() * RobotConstants.DRIVETRAIN_TRACK_WIDTH / 2.0; // in inches/100ms
        double left = translation.y()  - Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double right = translation.y() + Drivetrain.rpmToTicksPer100ms(Drivetrain.inchesPerSecondToRpm(arcLength * 10));
        double normal = translation.x();

        // System.out.println("[D] Forward Percentage: " + leftPercentage);
        // System.out.println("[D] Normal Percentage: " + sidewaysPercentage);

        return new DriveSignal(left, right, normal);
    }

    public static DriveSignal correctField(Translation translation, Rotation rotation, double fieldAngle) {
        return hDrive(translation.rotateBy(Rotation.fromDegrees(fieldAngle)), rotation);
    }
    */
}
