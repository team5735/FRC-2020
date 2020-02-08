/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.Twist;
import frc.lib.util.Util;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveJoystick extends CommandBase {

  private Drivetrain drivetrain;

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
    drivetrain.drive(new Twist(0.3 * Util.deadband(RobotContainer.subsystemController.leftStick.getX(), 0.2),
     0.3 * Util.deadband(RobotContainer.subsystemController.leftStick.getY(), 0.2), 
     Util.deadband(RobotContainer.subsystemController.rightStick.getX(), 0.1)));


    // System.out.println);
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
