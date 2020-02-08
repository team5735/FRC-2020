/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Pose;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.geometry.Rotation;
import frc.lib.trajectory.*;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

public class TrajectoryGenerator extends SubsystemBase {
    
    private final DrivetrainTrajectory drivetrain;

    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private TrajectorySet mTrajectorySet = null;

    public TrajectoryGenerator(DrivetrainTrajectory drivetrain) {
        this.drivetrain = drivetrain;
        // mMotionPlanner = new DriveMotionPlanner();
        generateTrajectories();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<PoseWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose> waypoints, final List<TimingConstraint<PoseWithCurvature>> constraints, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return drivetrain.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<PoseWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose> waypoints, final List<TimingConstraint<PoseWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return drivetrain.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel,
                max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle
    // of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
    // axis for LEFT)
    public static final Pose StartPose = new Pose(0.0, 0.0, Rotation.Identity);

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<PoseWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<PoseWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<PoseWithCurvature>> left;
            public final Trajectory<TimedState<PoseWithCurvature>> right;
        }

        public final MirroredTrajectory sideStartToNearScale;

        private TrajectorySet() {
            sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());
        }

        private Trajectory<TimedState<PoseWithCurvature>> getSideStartToNearScale() {
            List<Pose> waypoints = new ArrayList<>();
            waypoints.add(new Pose(0.0, 0.0, Rotation.fromDegrees(180.0)));
            waypoints.add(new Pose(10.0, 0.0, Rotation.fromDegrees(180.0)));
            // waypoints.add(new Pose(0.0, 0.0, Rotation.fromDegrees(180.0)));
            // waypoints.add(kSideStartPose.transformBy(Pose.fromTranslation(new
            // Translation(-120.0, 0.0))));
            // waypoints.add(kNearScaleEmptyPose);
            return generateTrajectory(true, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
