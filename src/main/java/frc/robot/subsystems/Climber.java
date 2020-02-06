/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.WinchSpinCommand;

public class Climber extends SubsystemBase {

  private final ElevatorMoveCommand c_elevator;
  private final WinchSpinCommand c_winchSpin;

  private final TalonSRX elevatorMaster;
  private final CANSparkMax winchMaster;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    c_elevator = new ElevatorMoveCommand(this);
    c_winchSpin = new WinchSpinCommand(this);

    elevatorMaster = new TalonSRX(1000);
    elevatorMaster.configFactoryDefault();

    winchMaster = new CANSparkMax(1000, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
