/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Pose;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Twist;
import frc.lib.util.DriveSignal;
import frc.robot.constants.RobotConstants;
import frc.robot.helper.HDriveHelper;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private PigeonIMU pigeon;
  public Rotation gyro_heading;
  private Rotation gyroOffset;
  private TalonSRX leftMaster, rightMaster, normalMaster;

  public Drive() {

    /**
     *
     * 
     * private TalonSRX leftMaster, rightMaster, leftFollower, rightFollower;
     * 
     * private TalonSRX leftMaster, rightMaster, rightFollower; private VictorSPX
     * leftFollower;
     * 
     */
    leftMaster = new TalonSRX(RobotConstants.LEFT_MASTER_ID);
    leftMaster.configFactoryDefault();
    leftMaster.setSensorPhase(true);
    leftMaster.selectProfileSlot(0, 0);
    leftMaster.config_kP(0, RobotConstants.LEFT_kP);
    leftMaster.config_kI(0, RobotConstants.LEFT_kI);
    leftMaster.config_kD(0, RobotConstants.LEFT_kD);
    leftMaster.config_kF(0, RobotConstants.LEFT_kF);
    leftMaster.configNeutralDeadband(0.04, 0);
    // leftFollower = new TalonSRX(Constants.DRIVETRAIN_LEFT_FOLLOWER_MOTOR_ID);
    // leftFollower.setInverted(false);
    // leftFollower.configFactoryDefault();
    // leftFollower.follow(leftMaster);
    // leftFollower.configNeutralDeadband(0.04, 0);

    rightMaster = new TalonSRX(RobotConstants.RIGHT_MASTER_ID);
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(true);
    rightMaster.setSensorPhase(true);
    rightMaster.selectProfileSlot(0, 0);
    rightMaster.config_kP(0, RobotConstants.RIGHT_kP);
    rightMaster.config_kI(0, RobotConstants.RIGHT_kI);
    rightMaster.config_kD(0, RobotConstants.RIGHT_kD);
    rightMaster.config_kF(0, RobotConstants.RIGHT_kF);
    rightMaster.configNeutralDeadband(0.04, 0);
    // rightFollower = new TalonSRX(Constants.DRIVETRAIN_RIGHT_FOLLOWER_MOTOR_ID);
    // rightFollower.configFactoryDefault();
    // rightFollower.setInverted(true);
    // rightFollower.follow(rightMaster);
    // rightFollower.configNeutralDeadband(0.04, 0);

    normalMaster = new TalonSRX(RobotConstants.RIGHT_MASTER_ID);
    normalMaster.configFactoryDefault();
    normalMaster.setInverted(true);
    normalMaster.setSensorPhase(true);
    normalMaster.selectProfileSlot(0, 0);
    normalMaster.config_kP(0, RobotConstants.NORMAL_kP);
    normalMaster.config_kI(0, RobotConstants.NORMAL_kI);
    normalMaster.config_kD(0, RobotConstants.NORMAL_kD);
    normalMaster.config_kF(0, RobotConstants.NORMAL_kF);
    normalMaster.configNeutralDeadband(0.04, 0);
    // normalFollower = new TalonSRX(Constants.DRIVETRAIN_RIGHT_FOLLOWER_MOTOR_ID);
    // normalFollower.configFactoryDefault();
    // normalFollower.setInverted(true);
    // normalFollower.follow(rightMaster);
    // normalFollower.configNeutralDeadband(0.04, 0);

    pigeon = new PigeonIMU(normalMaster); // TODO TALON HOST
  }

  @Override
  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  public void drive(Twist velocity) {
    drive(HDriveHelper.hDrive(velocity.getTranslation(), velocity.getRotation()));
  }

  public void drive(DriveSignal driveSignal) {
    leftMaster.set(ControlMode.Velocity, driveSignal.getLeft());
    rightMaster.set(ControlMode.Velocity, driveSignal.getRight());
    normalMaster.set(ControlMode.Velocity, driveSignal.getNormal());
  }

  public synchronized void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    normalMaster.setSelectedSensorPosition(0);
  }

  public void zeroSensors() {
    setHeading(Rotation.Identity);
    resetEncoders();
    // mAutoShift = true;
  }

  public synchronized Rotation getHeading() {
    return Rotation.fromDegrees(pigeon.getFusedHeading()).add(gyroOffset);
  }

  public synchronized void setHeading(Rotation heading) {
    System.out.println("SET HEADING: " + heading.degrees());

    gyroOffset = heading.add(Rotation.fromDegrees(pigeon.getFusedHeading()).inverse());
    System.out.println("Gyro offset: " + gyroOffset.degrees());
  }

  private static double rotationsToInches(double rotations) {
    return rotations * (RobotConstants.WHEELBASEANGULAR * Math.PI);
  }

  private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }

  private static double inchesToRotations(double inches) {
    return inches / (RobotConstants.WHEELBASEANGULAR * Math.PI);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
  }
}
