/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.Twist;
import frc.lib.util.Units;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveJoystick extends CommandBase {

  private Drivetrain drivetrain;

  private double maxVelocityInTicksper100ms = Units.rpmToTicks(Units.metersPerSecondToRpm(RobotConstants.MAX_VELOCITY));
  private double TURN_CONSTANT = Units.degreesToRadians(3); // maps [-1, 1] input to [-3deg, 3deg]/100ms

  /**
   * Creates a new DriveJoystick.
   */
  public DriveJoystick(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new Twist(maxVelocityInTicksper100ms * RobotContainer.subsystemController.leftStick.getXCubedWithDeadband(0.07),
    maxVelocityInTicksper100ms * RobotContainer.subsystemController.leftStick.getYCubedWithDeadband(0.07), 
     TURN_CONSTANT * RobotContainer.subsystemController.rightStick.getXCubedWithDeadband(0.07)));


    // System.out.println(unitgay);
    // drivetrain.drive(new DriveSignal(RobotContainer.subsystemController.getY(Hand.kLeft), 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
