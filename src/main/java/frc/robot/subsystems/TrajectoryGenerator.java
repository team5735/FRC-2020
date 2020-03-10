/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;

import frc.robot.constants.RobotConstants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryGenerator {

    public static Trajectory leftTrajectory;
    public static Trajectory rightTrajectory;

    private Waypoint[] points = new Waypoint[] {
        new Waypoint(3.138, -2.45, 0),
        new Waypoint(5.45, -0.718, 0),
        new Waypoint(8.04, -0.718, 0),
        // new Waypoint(1.5, 0.3, Pathfinder.d2r(0)),
        // new Waypoint(3, 1.75, Pathfinder.d2r(0))
        // new Waypoint(1.5, -1.5, Pathfinder.d2r(-90))
        // new Waypoint(3, 0, 0)    
        // new Waypoint(1.75, 0.2, 0), // positive ° = rotate left
        // new Waypoint(2.5, 0.2, 0),
    };

    public TrajectoryGenerator() {
        System.out.println("@@@@@@ GENERATING TRAJECTORIES @@@@@@");
        //                                                                                                               Time step, Max velocity,         max accel, max jerk
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 
                                                    0.02, 0.69 * RobotConstants.MAX_VELOCITY_DT, 0.8, 8);
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVETRAIN_TRACK_WIDTH);
        leftTrajectory = modifier.getLeftTrajectory();
        rightTrajectory = modifier.getRightTrajectory();

        // String fileName = "MiddleToTrench";
        // File left = new File(fileName + "_left.csv");
        // File right = new File(fileName + "_right.csv");
        // Pathfinder.writeToCSV(left, leftTrajectory);
        // Pathfinder.writeToCSV(right, rightTrajectory);
        // String fileName = "LineToTrench";
        // String fileName = "Straight";

        // File leftTraj = new File(Filesystem.getDeployDirectory() + "/" + fileName + "_left.csv");
        // File rightTraj = new File(Filesystem.getDeployDirectory() + "/" + fileName + "_right.csv");
        // try {
        //     leftTrajectory = Pathfinder.readFromCSV(leftTraj);
        //     rightTrajectory = Pathfinder.readFromCSV(rightTraj);
        // } catch(IOException exception) {
        //     exception.printStackTrace();
        // }
        
        // System.out.println("LEFT TRAJ: " + (leftTraj.exists()) + ", RIGHT TRAJ: " + (rightTraj.exists()));
    }
}
