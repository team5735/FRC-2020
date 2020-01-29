/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;

public class ExampleSubsystem extends SubsystemBase {

  /**
   * Creates a new ExampleSubsystem.
   */
  public ExampleSubsystem() {
    CommandScheduler.getInstance().setDefaultCommand(this, new ExampleCommand(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
