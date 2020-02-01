package frc.robot;

import frc.lib.geometry.Pose;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Twist;
import frc.robot.constants.RobotConstants;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */

public class Kinematics {

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (RobotConstants.DriveWheelRadiusInches * RobotConstants.DriveWheelRadiusInches);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    public static Twist forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist(dx, 0.0, delta_rotation_rads);
    }

    public static Twist forwardKinematics(Rotation prev_heading, double left_wheel_delta, double right_wheel_delta,
                                            Rotation current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist(dx, dy, prev_heading.inverse().rotateBy(current_heading).radians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist and previous rotation.
     */
    public static Pose integrateForwardKinematics(Pose current_pose,
                                                    Twist forward_kinematics) {
        return current_pose.transformBy(Pose.exp(forward_kinematics));
    }
}