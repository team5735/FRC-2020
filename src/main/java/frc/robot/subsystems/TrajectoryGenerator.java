/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class TrajectoryGenerator {

    public static Trajectory leftTrajectory;
    public static Trajectory rightTrajectory;

    public TrajectoryGenerator() {
        System.out.println("@@@@@@ GENERATING TRAJECTORIES @@@@@@");
        //                                                                                                               Time step, Max velocity,         max accel, max jerk
        /*Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, AutoConstants.MAX_VELOCITY, 0.5, 5.0);
        Trajectory trajectory = Pathfinder.generate(AutoConstants.points, config);
        modifier = new TankModifier(trajectory).modify(AutoConstants.TRACK_WIDTH);
        leftTrajectory = modifier.getLeftTrajectory();
        rightTrajectory = modifier.getRightTrajectory();*/
        // String fileName = "LineToTrench";
        String fileName = "Straight";

        File leftTraj = new File(Filesystem.getDeployDirectory() + "/" + fileName + "_left.csv");
        File rightTraj = new File(Filesystem.getDeployDirectory() + "/" + fileName + "_right.csv");
        try {
            leftTrajectory = Pathfinder.readFromCSV(leftTraj);
            rightTrajectory = Pathfinder.readFromCSV(rightTraj);
        } catch(IOException exception) {
            exception.printStackTrace();
        }
        
        System.out.println("LEFT TRAJ: " + (leftTraj.exists()) + ", RIGHT TRAJ: " + (rightTraj.exists()));
    }
}
