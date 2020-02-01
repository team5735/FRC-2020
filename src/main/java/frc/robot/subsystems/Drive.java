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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.geometry.Pose;
import frc.lib.geometry.PoseWithCurvature;
import frc.lib.geometry.Rotation;
import frc.lib.geometry.Twist;
import frc.lib.util.DriveSignal;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.constants.RobotConstants;
import frc.robot.helper.HDriveHelper;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private boolean overrideTrajectory = false;
  private PigeonIMU pigeon;
  public Rotation gyro_heading;
  private Rotation gyroOffset;

  public TimedState<PoseWithCurvature> path_setpoint;
  private TalonSRX leftMaster, rightMaster, normalMaster;
  private Pose error;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING // velocity PID control
  }

  private DriveControlState driveControlState;

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

  public void followPath() {
    if (driveControlState == DriveControlState.PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();
      

      Output output = update(now, Robot.robotState.getFieldToVehicle(now));
      // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
      // demand.right_feedforward_voltage / 12.0);

      error = error();
      // path_setpoint = setpoint();

      if (!overrideTrajectory) {
        drive(get
      } else { // BRAAAAAKEEE
        drive(DriveSignal.NEUTRAL);
      }
    } else {
      // Shuffleboard.reportError("Drive is not in path following state", false);
    }
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
    return Rotation.fromDegrees(pigeon.getFusedHeading()).rotateBy(gyroOffset);
  }

  public synchronized void setHeading(Rotation heading) {
    System.out.println("SET HEADING: " + heading.degrees());

    gyroOffset = heading.rotateBy(Rotation.fromDegrees(pigeon.getFusedHeading()).inverse());
    System.out.println("Gyro offset: " + gyroOffset.degrees());
  }

  public static class Output {
    public Output() {
    }

    public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
        double left_feedforward_voltage, double right_feedforward_voltage) {
      this.left_velocity = left_velocity;
      this.right_velocity = right_velocity;
      this.left_accel = left_accel;
      this.right_accel = right_accel;
      this.left_feedforward_voltage = left_feedforward_voltage;
      this.right_feedforward_voltage = right_feedforward_voltage;
    }

    public double left_velocity; // rad/s
    public double right_velocity; // rad/s

    public double left_accel; // rad/s^2
    public double right_accel; // rad/s^2

    public double left_feedforward_voltage;
    public double right_feedforward_voltage;

    public void flip() {
      double tmp_left_velocity = left_velocity;
      left_velocity = -right_velocity;
      right_velocity = -tmp_left_velocity;

      double tmp_left_accel = left_accel;
      left_accel = -right_accel;
      right_accel = -tmp_left_accel;

      double tmp_left_feedforward = left_feedforward_voltage;
      left_feedforward_voltage = -right_feedforward_voltage;
      right_feedforward_voltage = -tmp_left_feedforward;
    }
  }

  protected Output updatePID(DifferentialDrive.DriveDynamics dynamics, Pose current_state) {
    DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
    // Feedback on longitudinal error (distance).
    final double kPathKX = 5.0;
    final double kPathKY = 1.0;
    final double kPathKTheta = 5.0;
    adjusted_velocity.linear = dynamics.chassis_velocity.linear
        + kPathKX * Units.inches_to_meters(mError.getTranslation().x());
    adjusted_velocity.angular = dynamics.chassis_velocity.angular
        + dynamics.chassis_velocity.linear * kPathKY * Units.inches_to_meters(mError.getTranslation().y())
        + kPathKTheta * mError.getRotation().getRadians();

    double curvature = adjusted_velocity.angular / adjusted_velocity.linear;
    if (Double.isInfinite(curvature)) {
      adjusted_velocity.linear = 0.0;
      adjusted_velocity.angular = dynamics.chassis_velocity.angular;
    }

    // Compute adjusted left and right wheel velocities.
    final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
    final double left_voltage = dynamics.voltage.left
        + (wheel_velocities.left - dynamics.wheel_velocity.left) / mModel.left_transmission().speed_per_volt();
    final double right_voltage = dynamics.voltage.right
        + (wheel_velocities.right - dynamics.wheel_velocity.right) / mModel.right_transmission().speed_per_volt();

    return new Output(wheel_velocities.left, wheel_velocities.right, dynamics.wheel_acceleration.left,
        dynamics.wheel_acceleration.right, left_voltage, right_voltage);
  }

  public Output update(double timestamp, Pose current_state) {
    if (mCurrentTrajectory == null)
      return new Output();

    if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
      mLastTime = timestamp;
    }

    mDt = timestamp - mLastTime;
    mLastTime = timestamp;
    TrajectorySamplePoint<TimedState<PoseWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
    mSetpoint = sample_point.state();

    if (!mCurrentTrajectory.isDone()) {
      // Generate feedforward voltages.
      final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
      final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
      final double dcurvature_ds_m = Units
          .meters_to_inches(Units.meters_to_inches(mSetpoint.state().getDCurvatureDs()));
      final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
      final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
          new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m), new DifferentialDrive.ChassisState(
              acceleration_m, acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
      mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

      if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
        mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right,
            dynamics.wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage.left,
            dynamics.voltage.right);
      } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
        mOutput = updatePurePursuit(dynamics, current_state);
      } else if (mFollowerType == FollowerType.PID) {
        mOutput = updatePID(dynamics, current_state);
      } else if (mFollowerType == FollowerType.NONLINEAR_FEEDBACK) {
        mOutput = updateNonlinearFeedback(dynamics, current_state);
      }
    } else {
      // TODO Possibly switch to a pose stabilizing controller?
      mOutput = new Output();
    }
    return mOutput;
  }

  public void overrideTrajectory(boolean value) {
    overrideTrajectory = value;
  }

  public boolean isDone() {
    return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
  }

  public boolean isDoneWithTrajectory() {
    if (driveControlState != DriveControlState.PATH_FOLLOWING) {
      return false;
    }
    return this.isDone() || overrideTrajectory;
  }

  public Pose error() {
    return error;
  }

  private static double rotationsToInches(double rotations) {
    return rotations * (RobotConstants.WHEELBASEINCHES * Math.PI);
  }

  private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }

  private static double inchesToRotations(double inches) {
    return inches / (RobotConstants.WHEELBASEINCHES * Math.PI);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
  }
}
