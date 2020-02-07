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
import frc.lib.geometry.Translation;
import frc.lib.geometry.Rotation;
import frc.lib.trajectory.*;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.robot.RobotContainer;

public class TrajectoryGenerator extends SubsystemBase {
    
    private final Drivetrain drivetrain;

    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private TrajectorySet mTrajectorySet = null;

    public TrajectoryGenerator(Drivetrain drivetrain) {
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
    public static final Pose kSideStartPose = new Pose(0.0, 0.0, Rotation.fromDegrees(180.0));
    public static final Pose kNearScaleEmptyPose = new Pose(new Translation(253.0, 28.0),
            Rotation.fromDegrees(10 + 180.0));
    public static final Pose kNearScaleFullPose = new Pose(new Translation(253.0, 28.0 + 5.0),
            Rotation.fromDegrees(10.0 + 180.0));

    public static final Pose kNearScaleFullPose1 = new Pose(new Translation(253.0, 28.0 + 8.0),
            Rotation.fromDegrees(10.0 + 180.0));

    public static final Pose kNearScaleFullPose2 = new Pose(new Translation(253.0, 28.0 + 11.0 + 7.0),
            Rotation.fromDegrees(20.0 + 180.0));

    public static final Pose kNearScaleEndPose = new Pose(new Translation(220.0, 0.0),
            Rotation.fromDegrees(45.0));

    public static final Pose kFarScaleEmptyPose = new Pose(new Translation(256.0, 200.0),
            Rotation.fromDegrees(-10.0 + 180.0));
    public static final Pose kFarScaleFullPose = new Pose(new Translation(256.0, 200.0 - 5.0),
            Rotation.fromDegrees(-15.0 + 180.0));
    public static final Pose kFarScaleFullPose1 = new Pose(new Translation(256.0, 200.0 - 8.0),
            Rotation.fromDegrees(-20.0 + 180.0));
    public static final Pose kFarScaleFullPose2 = new Pose(new Translation(256.0, 200.0 - 11.0 - 10.0),
            Rotation.fromDegrees(-15.0 + 180.0));

    public static final Pose kCenterToIntake = new Pose(new Translation(-24.0, 0.0), Rotation.Identity);

    public static final Pose kNearCube1Pose = new Pose(new Translation(183.0, 46.0),
            Rotation.fromDegrees(180.0 - 25.0));
    public static final Pose kNearCube2Pose = new Pose(new Translation(180.0, 46.0 + 30.0 + 15.0),
            Rotation.fromDegrees(180.0 - 65.0));
    public static final Pose kNearCube3Pose = new Pose(new Translation(170.0, 46.0 + 30.0 * 2 + 20.0),
            Rotation.fromDegrees(180.0 - 60.0));

    public static final Pose kNearFence1Pose = kNearCube1Pose.transformBy(kCenterToIntake);
    public static final Pose kNearFence2Pose = kNearCube2Pose.transformBy(kCenterToIntake);
    public static final Pose kNearFence3Pose = kNearCube3Pose.transformBy(kCenterToIntake);

    public static final Pose kFarCube1Pose = new Pose(new Translation(185.0, 180.0),
            Rotation.fromDegrees(180.0 + 25.0));
    public static final Pose kFarCube2Pose = new Pose(new Translation(183.0, 180.0 - 30.0 - 15.0),
            Rotation.fromDegrees(180.0 + 65.0));
    public static final Pose kFarCube3Pose = new Pose(new Translation(174.0, 180.0 - 30.0 * 2 - 20.0),
            Rotation.fromDegrees(180.0 + 60.0));

    public static final Pose kFarFence1Pose = kFarCube1Pose.transformBy(kCenterToIntake);
    public static final Pose kFarFence2Pose = kFarCube2Pose.transformBy(kCenterToIntake);
    public static final Pose kFarFence3Pose = kFarCube3Pose.transformBy(kCenterToIntake);

    // STARTING IN CENTER
    public static final Pose kCenterStartPose = new Pose(0.0, -4.0, Rotation.fromDegrees(180.0));
    public static final Pose kSimpleSwitchStartPose = new Pose(0.0, -2.0, Rotation.fromDegrees(180.0));
    public static final Pose kRightSwitchPose = new Pose(new Translation(100.0, -60.0),
            Rotation.fromDegrees(0.0 + 180.0));
    public static final Pose kLeftSwitchPose = new Pose(new Translation(100.0, 60.0),
            Rotation.fromDegrees(0.0 + 180.0));

    public static final Pose kPyramidCubePose = new Pose(new Translation(82.0, 5.0),
            Rotation.fromDegrees(0.0 + 60.0)).transformBy(kCenterToIntake);
    public static final Pose kCenterPyramidCubePose = new Pose(new Translation(90.0, 0.0),
            Rotation.fromDegrees(0.0)).transformBy(kCenterToIntake);
    public static final Pose kPyramidCube1Pose = new Pose(new Translation(106.0, 3.0),
            Rotation.fromDegrees(0.0)).transformBy(kCenterToIntake);
    public static final Pose kPyramidCube2Pose = new Pose(new Translation(100.0, 6.0 - 2.0),
            Rotation.fromDegrees(0.0 + 60.0)).transformBy(kCenterToIntake);

    public static final Pose kSimpleSwitchEndPose = new Pose(new Translation(160, -106.0),
            Rotation.fromDegrees(180.0 + 0.0));

    public static final Pose kScalePoseLeft = new Pose(new Translation(253.0 + 8.0, -84.0),
            Rotation.fromDegrees(15.0 + 180.0));
    public static final Pose kScalePose1Left = new Pose(new Translation(253.0 + 8.0, -84.0),
            Rotation.fromDegrees(15.0 + 180.0));
    public static final Pose kCube1PoseLeft = new Pose(new Translation(183.0 + 6.0, -84.0 + 18.0),
            Rotation.fromDegrees(180.0 - 25.0));
    public static final Pose kFence1PoseLeft = kCube1PoseLeft.transformBy(kCenterToIntake);

    public static final Pose kScalePoseRight = new Pose(new Translation(253.0 + 8.0, -96.0),
            Rotation.fromDegrees(15.0 + 180.0));
    public static final Pose kScalePose1Right = new Pose(new Translation(253.0 + 8.0, -96.0),
            Rotation.fromDegrees(15.0 + 180.0));
    public static final Pose kCube1PoseRight = new Pose(new Translation(183.0 + 6.0, -96.0 + 18.0),
            Rotation.fromDegrees(180.0 - 25.0));
    public static final Pose kFence1PoseRight = kCube1PoseRight.transformBy(kCenterToIntake);

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
            waypoints.add(new Pose(-10.0, -4.0, Rotation.fromDegrees(180.0)));
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
