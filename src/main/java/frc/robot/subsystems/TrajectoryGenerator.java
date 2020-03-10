/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;

import frc.robot.constants.RobotConstants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryGenerator {

    public TrajectoryGenerator() {
    }

    // public static Trajectory[] generateTrajectoryWithWaypoints(Waypoint... waypoints) {
    //     ArrayList<Waypoint> points = new ArrayList<Waypoint>();

    //     for (Waypoint waypoint : waypoints) {
    //         points.add(waypoint);
    //     }
    //     return generateTrajectoryWithWaypoints(points.toArray());
    // }

    public static Trajectory[] generateTrajectoryWithWaypoints(Waypoint[] points) {
        Trajectory leftTrajectory;
        Trajectory rightTrajectory;

        long timenow = System.currentTimeMillis();
        System.out.println("@@@@@@ GENERATING TRAJECTORIES @@@@@@" + timenow);

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 
                                                    0.02, 0.69 * RobotConstants.MAX_VELOCITY_DT, 0.8, 8);
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVETRAIN_TRACK_WIDTH);
        leftTrajectory = modifier.getLeftTrajectory();
        rightTrajectory = modifier.getRightTrajectory();

        System.out.println(System.currentTimeMillis() - timenow);
        return new Trajectory[] {leftTrajectory, rightTrajectory};
    }
}
