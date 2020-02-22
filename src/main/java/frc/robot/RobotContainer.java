/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.controllers.BobXboxController;
import frc.robot.commandgroups.SixBallAutoCommand;
import frc.robot.commandgroups.TurnAndShootCommand;
import frc.robot.commands.CancelAllCommand;
import frc.robot.commands.drivetrain.ChangeDriveMode;
import frc.robot.commands.drivetrain.ResetGyroAngle;
import frc.robot.commands.intake.AngleIntakeCommand;
import frc.robot.commands.intake.FeedShooterCommand;
import frc.robot.commands.intake.IntakeBallCommand;
import frc.robot.commands.intake.MoveConveyorCommand;
import frc.robot.commands.intake.ZeroIntakeCommand;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrajectoryGenerator;
import frc.robot.subsystems.Vision;

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
	public final Vision vision = new Vision();
	
	public static final BobXboxController driverController = new BobXboxController(0);
	public static final BobXboxController subsystemController = new BobXboxController(1);
	
	/**
	* The container for the robot.  Contains subsystems, OI devices, and commands.
	*/
	public RobotContainer() {
		// Configure the button bindings
		configureDriverBindings();
		configureSubsystemBindings();
	}
	
	private void configureDriverBindings() {
		// subsystemController.xButton.whenPressed(new ColorSpinCommand(colorSpinner, 4));
		// subsystemController.bButton.whenPressed(new ColorMatchCommand(colorSpinner, colorMatcher));
		// driverController.aButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_DEPLOYED));
		// driverController.bButton.whenPressed(new AngleIntakeCommand(intake, RobotConstants.INTAKE_POSITION_RETRACTED));
		driverController.yButton.whenPressed(new ResetGyroAngle(drivetrain));
		driverController.xButton.whenPressed(new ChangeDriveMode(drivetrain));
		
		driverController.rightTriggerButton.whileActiveContinuous(new IntakeBallCommand(intake));
		driverController.leftTriggerButton.whileActiveContinuous(new IntakeBallCommand(intake));

		// driverController.startButton.whenPressed(new ZeroIntakeCommand(intake));
		//   trajectoryGenerator.getTrajectorySet().sideStartToNearScale.left, (DrivetrainTrajectory)drivetrain));
	}

	private void configureSubsystemBindings() {
		// subsystemController.aButton.whenPressed(new RampShooterCommand(shooter, 3750));
		// subsystemController.aButton.whenReleased(new RampShooterCommand(shooter, 0));

		subsystemController.aButton.whenPressed(new TurnAndShootCommand(vision, drivetrain, intake, shooter));
		// subsystemController.bButton.whenPressed();
		// subsystemController.yButton.whenPressed();

		// subsystemController.Dpad.Up.whenPressed(new RampShooterCommand(shooter, RobotConstants.FLYWHEEL_PRESET_LINE));
		// subsystemController.Dpad.Right.whenPressed(new RampShooterCommand(shooter, RobotConstants.FLYWHEEL_PRESET_TRENCH));
		// subsystemController.Dpad.Left.whenPressed(new RampShooterCommand(shooter, RobotConstants.FLYWHEEL_PRESET_BEHINDCOLORWHEEL));
		// subsystemController.Dpad.Down.whenPressed(new RampShooterCommand(shooter, 0));
		
		subsystemController.rightBumper.whileActiveContinuous(new MoveConveyorCommand(intake, false));
		subsystemController.leftBumper.whileActiveContinuous(new MoveConveyorCommand(intake, true));

		subsystemController.rightTriggerButton.whileActiveContinuous(new FeedShooterCommand(intake, false));
		subsystemController.leftTriggerButton.whileActiveContinuous(new FeedShooterCommand(intake, true));
		
	}
	
	/**
	* Use this to pass the autonomous command to the main {@link Robot} class.
	*
	* @return the command to run in autonomous
	*/
	public Command getAutonomousCommand() {
		return new SixBallAutoCommand(vision, drivetrain, intake, shooter);
	}
	
}
