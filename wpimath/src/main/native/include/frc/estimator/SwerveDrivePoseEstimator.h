// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <fmt/format.h>
#include <wpi/SymbolExports.h>
#include <wpi/array.h>
#include <wpi/timestamp.h>

#include "frc/EigenCore.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/interpolation/TimeInterpolatableBuffer.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "units/time.h"

namespace frc {

/**
 * This class wraps Swerve Drive Odometry to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is
 * intended to be a drop-in for SwerveDriveOdometry.
 *
 * Update() should be called every robot loop.
 *
 * AddVisionMeasurement() can be called as infrequently as you want; if you
 * never call it, then this class will behave as regular encoder
 * odometry.
 *
 * The state-space system used internally has the following states (x) and
 * outputs (y):
 *
 * <strong> x = [x, y, theta]ᵀ </strong> in the field coordinate
 * system containing x position, y position, and heading.
 *
 * <strong> y = [x, y, theta]ᵀ </strong> from vision containing x position, y
 * position, and heading.
 */
template <size_t NumModules>
class SwerveDrivePoseEstimator {
 public:
  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics               A correctly-configured kinematics object
   *                                 for your drivetrain.
   * @param gyroAngle                The current gyro angle.
   * @param modulePositions          The current distance and rotation
   *                                 measurements of the swerve modules.
   * @param initialPose              The starting pose estimate.
   * @param stateStdDevs             Standard deviations of model states.
   *                                 Increase these numbers to trust your
   *                                 model's state estimates less. This matrix
   *                                 is in the form [x, y, theta]ᵀ, with units
   *                                 in meters and radians.
   * @param visionMeasurementStdDevs Standard deviations of the vision
   *                                 measurements. Increase these numbers to
   *                                 trust global measurements from vision
   *                                 less. This matrix is in the form
   *                                 [x, y, theta]ᵀ, with units in meters and
   *                                 radians.
   */
  SwerveDrivePoseEstimator(
      SwerveDriveKinematics<NumModules>& kinematics,
      const Rotation2d& gyroAngle,
      const wpi::array<SwerveModulePosition, NumModules>& modulePositions,
      const Pose2d& initialPose, const wpi::array<double, 3>& stateStdDevs,
      const wpi::array<double, 3>& visionMeasurementStdDevs)
      : m_kinematics{kinematics},
        m_odometry{kinematics, gyroAngle, modulePositions, initialPose} {
    for (size_t i = 0; i < 3; ++i) {
      m_q[i] = stateStdDevs[i] * stateStdDevs[i];
    }

    SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Resets the robot's position on the field.
   *
   * The gyroscope angle does not need to be reset in the user's robot code.
   * The library automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle       The angle reported by the gyroscope.
   * @param modulePositions The current distance and rotation measurements of
   *                        the swerve modules.
   * @param pose            The position on the field that your robot is at.
   */
  void ResetPosition(
      const Rotation2d& gyroAngle,
      const wpi::array<SwerveModulePosition, NumModules>& modulePositions,
      const Pose2d& pose) {
    // Reset state estimate and error covariance
    m_odometry.ResetPosition(gyroAngle, modulePositions, pose);
    m_poseBuffer.Clear();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  Pose2d GetEstimatedPosition() const { return m_odometry.GetPose(); }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used
   * to change trust in vision measurements after the autonomous period, or to
   * change trust as distance to a vision target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision
   *                                 measurements. Increase these numbers to
   *                                 trust global measurements from vision
   *                                 less. This matrix is in the form
   *                                 [x, y, theta]ᵀ, with units in meters and
   *                                 radians.
   */
  void SetVisionMeasurementStdDevs(
      const wpi::array<double, 3>& visionMeasurementStdDevs) {
    wpi::array<double, 3> r{wpi::empty_array};
    for (size_t i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs[i] * visionMeasurementStdDevs[i];
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (size_t row = 0; row < 3; ++row) {
      if (m_q[row] == 0.0) {
        m_visionK(row, row) = 0.0;
      } else {
        m_visionK(row, row) =
            m_q[row] / (m_q[row] + std::sqrt(m_q[row] * r[row]));
      }
    }
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate while still accounting for measurement noise.
   *
   * This method can be called as infrequently as you want, as long as you are
   * calling Update() every loop.
   *
   * To promote stability of the pose estimate and make it robust to bad vision
   * data, we recommend only adding vision measurements that are already within
   * one meter or so of the current pose estimate.
   *
   * @param visionRobotPose The pose of the robot as measured by the vision
   *                        camera.
   * @param timestamp       The timestamp of the vision measurement in seconds.
   *                        Note that if you don't use your own time source by
   *                        calling UpdateWithTime() then you must use a
   *                        timestamp with an epoch since FPGA startup
   *                        (i.e. the epoch of this timestamp is the same
   *                        epoch as Timer#GetFPGATimestamp.) This means
   *                        that you should use Timer#GetFPGATimestamp as your
   *                        time source or sync the epochs.
   */
  void AddVisionMeasurement(const Pose2d& visionRobotPose,
                            units::second_t timestamp) {
    // Step 1: Get the estimated pose from when the vision measurement was made.
    auto sample = m_poseBuffer.Sample(timestamp);

    if (!sample.has_value()) {
      return;
    }

    // Step 2: Measure the twist between the odometry pose and the vision pose
    auto twist = sample.value().pose.Log(visionRobotPose);

    // Step 3: We should not trust the twist entirely, so instead we scale this
    // twist by a Kalman gain matrix representing how much we trust vision
    // measurements compared to our current pose.
    frc::Vectord<3> k_times_twist =
        m_visionK * frc::Vectord<3>{twist.dx.value(), twist.dy.value(),
                                    twist.dtheta.value()};

    // Step 4: Convert back to Twist2d
    Twist2d scaledTwist{units::meter_t{k_times_twist(0)},
                        units::meter_t{k_times_twist(1)},
                        units::radian_t{k_times_twist(2)}};

    // Step 5: Apply scaled twist to the latest pose
    auto estimatedPose = GetEstimatedPosition().Exp(scaledTwist);

    // Step 6: Apply new pose to odometry
    m_odometry.ResetPosition(sample.value().gyroAngle, sample.value().modulePostions,
                            estimatedPose);

    auto internal_buf = m_poseBuffer.GetInternalStructure();

    auto upper_bound = std::lower_bound(
      internal_buf.begin(), internal_buf.end(), timestamp,
      [](const auto& pair, auto t) { return t > pair.first; });

    for(auto entry = upper_bound; entry != internal_buf.end(); entry++) {
      UpdateWithTime(
        entry->first,
        entry->second.gyroAngle,
        entry->second.modulePostions
      );
    }
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate while still accounting for measurement noise.
   *
   * This method can be called as infrequently as you want, as long as you are
   * calling Update() every loop.
   *
   * To promote stability of the pose estimate and make it robust to bad vision
   * data, we recommend only adding vision measurements that are already within
   * one meter or so of the current pose estimate.
   *
   * Note that the vision measurement standard deviations passed into this
   * method will continue to apply to future measurements until a subsequent
   * call to SetVisionMeasurementStdDevs() or this method.
   *
   * @param visionRobotPose          The pose of the robot as measured by the
   *                                 vision camera.
   * @param timestamp                The timestamp of the vision measurement in
   *                                 seconds. Note that if you don't use your
   *                                 own time source by calling
   *                                 UpdateWithTime(), then you must use a
   *                                 timestamp with an epoch since FPGA startup
   *                                 (i.e. the epoch of this timestamp is the
   *                                 same epoch as
   *                                 frc::Timer::GetFPGATimestamp(). This means
   *                                 that you should use
   *                                 frc::Timer::GetFPGATimestamp() as your
   *                                 time source in this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision
   *                                 measurements. Increase these numbers to
   *                                 trust global measurements from vision
   *                                 less. This matrix is in the form
   *                                 [x, y, theta]ᵀ, with units in meters and
   *                                 radians.
   */
  void AddVisionMeasurement(
      const Pose2d& visionRobotPose, units::second_t timestamp,
      const wpi::array<double, 3>& visionMeasurementStdDevs) {
    SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
    AddVisionMeasurement(visionRobotPose, timestamp);
  }

  /**
   * Updates the Kalman Filter using only wheel encoder information. This should
   * be called every loop.
   *
   * @param gyroAngle       The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of
   *                        the swerve modules.
   * @return The estimated robot pose in meters.
   */
  Pose2d Update(
      const Rotation2d& gyroAngle,
      const wpi::array<SwerveModulePosition, NumModules>& modulePositions) {
    return UpdateWithTime(units::microsecond_t(wpi::Now()), gyroAngle,
                          modulePositions);
  }

  /**
   * Updates the Kalman Filter using only wheel encoder information. This should
   * be called every loop.
   *
   * @param currentTime     Time at which this method was called, in seconds.
   * @param gyroAngle       The current gyro angle.
   * @param modulePositions The current distance travelled and rotations of
   *                        the swerve modules.
   * @return The estimated robot pose in meters.
   */
  Pose2d UpdateWithTime(
      units::second_t currentTime, const Rotation2d& gyroAngle,
      const wpi::array<SwerveModulePosition, NumModules>& modulePositions) {
    m_odometry.Update(gyroAngle, modulePositions);

    wpi::array<SwerveModulePosition, NumModules> internalModulePositions{wpi::empty_array};

    for (size_t i = 0; i < NumModules; i++) {
      internalModulePositions[i].distance = modulePositions[i].distance;
      internalModulePositions[i].angle = modulePositions[i].angle;
    }

    m_poseBuffer.AddSample(currentTime, {m_kinematics, GetEstimatedPosition(), gyroAngle, internalModulePositions});

    return GetEstimatedPosition();
  }

 private:
  struct InterpolationRecord {
    SwerveDriveKinematics<NumModules> &kinematics;
    Pose2d pose;
    Rotation2d gyroAngle;
    wpi::array<SwerveModulePosition, NumModules> modulePostions;

    /**
     * Checks equality between this InterpolationRecord and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const InterpolationRecord& other) const {
      return this->pose == other.pose && this->gyroAngle == other.gyroAngle && this->modulePostions == other.modulePostions;
    }

    /**
     * Checks inequality between this InterpolationRecord and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are not equal.
     */
    bool operator!=(const InterpolationRecord& other) const {
      return this->pose != other.pose || this->gyroAngle != other.gyroAngle || this->modulePostions != other.modulePostions;
    }

    /**
     * Interpolates between two InterpolationRecords.
     *
     * @param endValue The end value for the interpolation.
     * @param i The interpolant (fraction).
     *
     * @return The interpolated state.
     */
    InterpolationRecord Interpolate(InterpolationRecord endValue, double i) const {
      if (i < 0) {
        return *this;
      } else if (i > 1) {
        return endValue;
      } else {
        wpi::array<SwerveModulePosition, NumModules> wheels_lerp{wpi::empty_array};
        wpi::array<SwerveModulePosition, NumModules> wheels_delta{wpi::empty_array};


        for (size_t i = 0; i < NumModules; i++) {
          wheels_lerp[i].distance = wpi::Lerp(modulePostions[i].distance, endValue.modulePostions[i].distance, i);
          wheels_lerp[i].angle = wpi::Lerp(modulePostions[i].angle, endValue.modulePostions[i].angle, i);

          wheels_delta[i].distance = wpi::Lerp(modulePostions[i].distance, endValue.modulePostions[i].distance, i) - modulePostions[i].distance;
          wheels_delta[i].angle = wpi::Lerp(modulePostions[i].angle, endValue.modulePostions[i].angle, i);
        }

        auto gyro = wpi::Lerp(this->gyroAngle, endValue.gyroAngle, i);

        auto twist = kinematics.ToTwist2d(wheels_delta);
        twist.dtheta = (gyro - gyroAngle).Radians();


        return {kinematics, pose.Exp(twist), gyro, wheels_lerp}; 
      }
    };
  };

  SwerveDriveKinematics<NumModules> &m_kinematics;
  SwerveDriveOdometry<NumModules> m_odometry;
  wpi::array<double, 3> m_q{wpi::empty_array};
  Eigen::Matrix3d m_visionK = Eigen::Matrix3d::Zero();

  TimeInterpolatableBuffer<InterpolationRecord> m_poseBuffer{1.5_s, [](const InterpolationRecord &start, const InterpolationRecord &end, double t) { return start.Interpolate(end, t);}};
};

extern template class EXPORT_TEMPLATE_DECLARE(WPILIB_DLLEXPORT)
    SwerveDrivePoseEstimator<4>;

}  // namespace frc
