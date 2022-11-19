// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.estimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * This class wraps an {@link UnscentedKalmanFilter Unscented Kalman Filter} to fuse
 * latency-compensated vision measurements with swerve drive encoder velocity measurements. It will
 * correct for noisy measurements and encoder drift. It is intended to be an easy but more accurate
 * drop-in for {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry}.
 *
 * <p>The generic arguments to this class define the size of the state, input and output vectors
 * used in the underlying {@link UnscentedKalmanFilter Unscented Kalman Filter}. {@link Num States}
 * must be equal to the module count + 3. {@link Num Inputs} must be equal to the module count + 3.
 * {@link Num Outputs} must be equal to the module count + 1.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop. If your loops are
 * faster or slower than the default of 20 ms, then you should change the nominal delta time using
 * the secondary constructor: {@link SwerveDrivePoseEstimator#SwerveDrivePoseEstimator(Nat, Nat,
 * Nat, Rotation2d, SwerveModulePosition[], Pose2d, SwerveDriveKinematics, Matrix, Matrix, Matrix,
 * double)}.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave mostly like regular encoder odometry.
 *
 * <p>The state-space system used internally has the following states (x), inputs (u), and outputs
 * (y):
 *
 * <p><strong> x = [x, y, theta, s_0, ..., s_n]ᵀ </strong> in the field coordinate system containing
 * x position, y position, and heading, followed by the distance travelled by each wheel.
 *
 * <p><strong> u = [v_x, v_y, omega, v_0, ... v_n]ᵀ </strong> containing x velocity, y velocity, and
 * angular rate in the field coordinate system, followed by the velocity measured at each wheel.
 *
 * <p><strong> y = [x, y, theta]ᵀ </strong> from vision containing x position, y position, and
 * heading; or <strong> y = [theta, s_0, ..., s_n]ᵀ </strong> containing gyro heading, followed by
 * the distance travelled by each wheel.
 */
public class SwerveDrivePoseEstimator {
  private final SwerveDriveOdometry m_odometry;
  private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer;

  private SwerveModulePosition[] m_prevModulePositions;
  private final int m_numModules;

  private final double m_nominalDt; // Seconds

  private Rotation2d m_previousGyroAngle;

  private final Matrix<N3, N1> m_stateStdDevs;

  private Matrix<N3, N3> m_visionK;


  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs Standard deviations of model states. Increase these numbers to trust your
   *     model's state estimates less. This matrix is in the form [x, y, theta, s_0, ... s_n]ᵀ, with
   *     units in meters and radians, then meters.
   * @param localMeasurementStdDevs Standard deviations of the encoder and gyro measurements.
   *     Increase these numbers to trust sensor readings from encoders and gyros less. This matrix
   *     is in the form [theta, s_0, ... s_n], with units in radians followed by meters.
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   * @param nominalDtSeconds The time in seconds between each robot loop.
   */
  public SwerveDrivePoseEstimator(
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      SwerveDriveKinematics kinematics,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs,
      double nominalDtSeconds) {

    m_nominalDt = nominalDtSeconds;
    m_numModules = modulePositions.length;

    m_previousGyroAngle = gyroAngle;
    m_prevModulePositions = new SwerveModulePosition[m_numModules];
    for (int i = 0; i < m_numModules; i++) {
      m_prevModulePositions[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, null);
    }


    m_poseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    m_stateStdDevs = stateStdDevs;

    // Initialize vision K
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);

    m_odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    var visionObserver =
        new KalmanFilter<>(
            Nat.N3(),
            Nat.N3(),
            new LinearSystem<>(
              Matrix.eye(Nat.N3()), 
              new Matrix<>(Nat.N3(), Nat.N3()),
              Matrix.eye(Nat.N3()), 
              new Matrix<>(Nat.N3(), Nat.N3())),
            m_stateStdDevs,
            visionMeasurementStdDevs,
            m_nominalDt);

    m_visionK = visionObserver.getK();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    // Reset state estimate and error covariance
    m_odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
    m_poseBuffer.clear();
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the Unscented Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose
   * estimate while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime}
   *     then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
   *     timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
   *     Timer.getFPGATimestamp as your time source or sync the epochs.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    /**
     * @Jordan - 3538 Mentor, CSA I got a report of instability in the pose estimator UKF again in
     * the PhotonVision discord, so I have an idea: since the state and measurement models are all
     * linear, we could use SwerveDriveOdometry internally for predictions, compute a steady-state K
     * in the constructor via the KalmanFilter class, and do the vision correction manually. The
     * correction would be current state = previous state + K *
     * actualPoseMeasurement.RelativeTo(predictedPoseMeasurement) for some definition of
     * multiplication of pose by K (twist for interpolation?). currrent = prev + K * (measurement -
     * prev) current = prev + K * measurement - K * prev) current = (1 - K) * prev + K * measurement
     */

    
    var sample = m_poseBuffer.getSample(timestampSeconds);

    if (sample.isEmpty()) {
      return;
    }

    var twist = sample.get().log(visionRobotPoseMeters);

    var k_times_twist = m_visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    var scaled_twist = new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

    var est_pose = getEstimatedPosition().exp(scaled_twist);

    m_odometry.resetPosition(m_previousGyroAngle, m_prevModulePositions, est_pose);
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose
   * estimate while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * SwerveDrivePoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link SwerveDrivePoseEstimator#updateWithTime}
   *     then you must use a timestamp with an epoch since FPGA startup (i.e. the epoch of this
   *     timestamp is the same epoch as Timer.getFPGATimestamp.) This means that you should use
   *     Timer.getFPGATimestamp as your time source in this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust global measurements from vision less. This matrix is in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
   * called every loop, and the correct loop period must be passed into the constructor of this
   * class.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return updateWithTime(WPIUtilJNI.now() * 1.0e-6, gyroAngle, modulePositions);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder information. This should be
   * called every loop, and the correct loop period must be passed into the constructor of this
   * class.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param moduleStates The current velocities and rotations of the swerve modules.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    if (modulePositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    m_poseBuffer.addSample(currentTimeSeconds, getEstimatedPosition());
    m_odometry.update(gyroAngle, modulePositions);

    m_previousGyroAngle = gyroAngle;
    for (int i = 0; i < m_numModules; i++) {
      m_prevModulePositions[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, null);
    }

    return getEstimatedPosition();
  }
}
