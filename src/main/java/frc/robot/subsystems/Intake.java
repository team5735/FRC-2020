/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AngleIntakeCommand;
import frc.robot.commands.IntakeBallCommand;

public class Intake extends SubsystemBase {

  private final TalonSRX intakeMaster, intakeAngleMaster;

  private final IntakeBallCommand c_intakeBall;
  private final AngleIntakeCommand c_angleIntake;

  /**
   * Creates a new Climber.
   */
  public Intake() {
    c_intakeBall = new IntakeBallCommand(this);
    c_angleIntake = new AngleIntakeCommand(this);

    intakeMaster = new TalonSRX(1000);
    intakeMaster.configFactoryDefault();

    intakeAngleMaster = new TalonSRX(1000);
    intakeAngleMaster.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
