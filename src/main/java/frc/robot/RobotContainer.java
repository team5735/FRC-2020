/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.BobXboxController;
import frc.robot.commands.AngleIntakeCommand;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.DriveFollowTrajectory;
import frc.robot.commands.IntakeBallCommand;
import frc.robot.commands.MoveConveyorCommand;
import frc.robot.commands.ResetGyroAngle;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ColorMatcher colorMatcher = new ColorMatcher();
  public final ColorSpinner colorSpinner = new ColorSpinner();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Climber climber = new Climber();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final TrajectoryGenerator trajectoryGenerator = new TrajectoryGenerator();
  
  public static final BobXboxController subsystemController = new BobXboxController(0);
  
  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
      // subsystemController.xButton.whenPressed(new ColorSpinCommand(colorSpinner, 4));
      // subsystemController.bButton.whenPressed(new ColorMatchCommand(colorSpinner, colorMatcher));
      subsystemController.yButton.whenPressed(new ResetGyroAngle(drivetrain));
      subsystemController.xButton.whenPressed(new ChangeDriveMode(drivetrain));

      subsystemController.aButton.whenPressed(new ShootCommand(shooter, 4500));
      subsystemController.aButton.whenReleased(new ShootCommand(shooter, 0));

      subsystemController.rightTriggerButton.toggleWhenActive(new IntakeBallCommand(intake, subsystemController.triggers.getRight(), false));
      subsystemController.leftTriggerButton.toggleWhenActive(new IntakeBallCommand(intake, subsystemController.triggers.getLeft(), true));

      subsystemController.rightBumper.toggleWhenActive(new MoveConveyorCommand(intake, false));
      subsystemController.leftBumper.toggleWhenActive(new MoveConveyorCommand(intake, true));

      subsystemController.Dpad.Up.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_RETRACTED));
      subsystemController.Dpad.Down.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_DEPLOYED));
      //   trajectoryGenerator.getTrajectorySet().sideStartToNearScale.left, (DrivetrainTrajectory)drivetrain));
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      return new DriveFollowTrajectory(drivetrain);
    }
    
  }
  