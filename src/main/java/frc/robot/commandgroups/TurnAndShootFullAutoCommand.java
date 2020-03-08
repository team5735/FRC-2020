package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.RampShooterCommand;
import frc.robot.commands.shooter.StopFlywheel;
import frc.robot.commands.vision.TurnOffLimelightCommand;
import frc.robot.commands.vision.TurnToTargetCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Banana;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Drivetrain.DriveMode;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TurnAndShootFullAutoCommand extends SequentialCommandGroup {

    private Vision vision;
    private Drivetrain drivetrain;
    private Feeder feeder;
    private Conveyer conveyer;
    private IntakeArm intakeArm;
    private Shooter shooter;
    private Banana banana;


    /**
     * Turn to target, ramp up shooter, feed the shooter, and go!
     * @param vision
     * @param drivetrain
     * @param shooter
     */
    public TurnAndShootFullAutoCommand(Vision vision, Drivetrain drivetrain, Feeder feeder, Conveyer conveyer, IntakeArm intakeArm, Shooter shooter, Banana banana) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.conveyer = conveyer;
        this.intakeArm = intakeArm;
        this.shooter = shooter;
        this.banana = banana;
        
        TurnToTargetCommand turnToTargetCommand = new TurnToTargetCommand(vision, drivetrain);
        
        addCommands(
            // https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html
            new ParallelCommandGroup(
                turnToTargetCommand,
                new SequentialCommandGroup(
                    new RampShooterCommand(shooter, vision, banana, feeder, 3600),
                    new WaitUntilCommand(turnToTargetCommand),
                    new RampShooterCommand(shooter, vision, banana, feeder, true),
                    new ShootBallCommand(feeder, conveyer, intakeArm, shooter, false),
                    new ShootBallCommand(feeder, conveyer, intakeArm, shooter, false),
                    new ShootBallCommand(feeder, conveyer, intakeArm, shooter, false),
                    new StopFlywheel(shooter),
                    new TurnOffLimelightCommand(vision)
                )
            )
        );
    } 

    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setDriveMode(DriveMode.DISABLED);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.setDriveMode(DriveMode.STATIC_DRIVE);
    }
    
}