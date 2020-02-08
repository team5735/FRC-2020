/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.*;

public class Shooter extends SubsystemBase {

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    CommandScheduler.getInstance().setDefaultCommand(this, new ShootCommand(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
