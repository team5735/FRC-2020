/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AngleIntakeCommand;
import frc.robot.commands.IntakeBallCommand;

public class Intake extends SubsystemBase {

  private final IntakeBallCommand c_intakeBall;
  private final AngleIntakeCommand c_angleIntake;

  private final TalonSRX intakeArm;
  private final VictorSPX intakeRoller;

  /**
   * Creates a new Intake.
   */
  public Intake() {    

    intakeArm = new TalonSRX(1000);
    intakeArm.configFactoryDefault();

    intakeRoller = new VictorSPX(1000);
    intakeRoller.configFactoryDefault();

    c_intakeBall = new IntakeBallCommand(this);
    c_angleIntake = new AngleIntakeCommand(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinRoller(double speed, boolean inverted) {
    if (inverted) {
      intakeRoller.set(ControlMode.PercentOutput, speed);
    } else {
      intakeRoller.set(ControlMode.PercentOutput, -speed);
    }
  }
}
