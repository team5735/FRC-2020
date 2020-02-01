/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helper;

import frc.lib.geometry.Rotation;
import frc.lib.geometry.Translation;
import frc.lib.util.DriveSignal;
import frc.robot.constants.RobotConstants;

/**
 * Add your docs here.
 */
public class HDriveHelper {


    // translation is direction of travel with magnitute
    // rotation is rotating 

    public static DriveSignal xyLockedDrive(Translation translation) {
        return hDrive(translation, Rotation.Identity);
    }

    public static DriveSignal hDrive(Translation translation, Rotation rotation) {
        double left = translation.y()  - rotation.radians() * RobotConstants.WHEELBASEINCHES;
        double right = translation.y() + rotation.radians() * RobotConstants.WHEELBASEINCHES;
        double normal = translation.x();

        // System.out.println("[D] Forward Percentage: " + leftPercentage);
        // System.out.println("[D] Normal Percentage: " + sidewaysPercentage);

        return new DriveSignal(left, right, normal);
    }

    public static DriveSignal correctField(Translation translation, Rotation rotation, double fieldAngle) {
        return hDrive(translation.rotateBy(Rotation.fromDegrees(fieldAngle)), rotation);
    }
}
