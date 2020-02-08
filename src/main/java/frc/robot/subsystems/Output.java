/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

public class Output {
    public Output() {
    }

    public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
            double left_feedforward_voltage, double right_feedforward_voltage) {
        this.left_velocity = left_velocity;
        this.right_velocity = right_velocity;
        this.left_accel = left_accel;
        this.right_accel = right_accel;
        this.left_feedforward_voltage = left_feedforward_voltage;
        this.right_feedforward_voltage = right_feedforward_voltage;
    }

    public double left_velocity; // rad/s
    public double right_velocity; // rad/s

    public double left_accel; // rad/s^2
    public double right_accel; // rad/s^2

    public double left_feedforward_voltage;
    public double right_feedforward_voltage;

    public void flip() {
        double tmp_left_velocity = left_velocity;
        left_velocity = -right_velocity;
        right_velocity = -tmp_left_velocity;

        double tmp_left_accel = left_accel;
        left_accel = -right_accel;
        right_accel = -tmp_left_accel;

        double tmp_left_feedforward = left_feedforward_voltage;
        left_feedforward_voltage = -right_feedforward_voltage;
        right_feedforward_voltage = -tmp_left_feedforward;
    }
}
