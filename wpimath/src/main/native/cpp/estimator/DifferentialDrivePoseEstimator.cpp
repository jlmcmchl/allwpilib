// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/estimator/DifferentialDrivePoseEstimator.h"

#include <wpi/timestamp.h>

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/AngleStatistics.h"

using namespace frc;

DifferentialDrivePoseEstimator::InterpolationRecord DifferentialDrivePoseEstimator::InterpolationRecord::Interpolate(DifferentialDriveKinematics &kinematics, InterpolationRecord endValue,
                                                 double i) const {
  if (i < 0) {
    return *this;
  } else if (i > 1) {
    return endValue;
  } else {
    auto left = wpi::Lerp(this->leftDistance, endValue.leftDistance, i);
    auto right = wpi::Lerp(this->rightDistance, endValue.rightDistance, i);
    auto gyro = wpi::Lerp(this->gyroAngle, endValue.gyroAngle, i);

    auto twist = kinematics.ToTwist2d(left - leftDistance, right - rightDistance);
    twist.dtheta = (gyro - gyroAngle).Radians();


    return {pose.Exp(twist), wpi::Lerp(this->gyroAngle, endValue.gyroAngle, i), left, right}; 
  }
}

DifferentialDrivePoseEstimator::DifferentialDrivePoseEstimator(
    DifferentialDriveKinematics &kinematics,
    const Rotation2d& gyroAngle, units::meter_t leftDistance,
    units::meter_t rightDistance, const Pose2d& initialPose,
    const wpi::array<double, 3>& stateStdDevs,
    const wpi::array<double, 3>& visionMeasurementStdDevs)
    : m_kinematics{kinematics},
      m_odometry{gyroAngle, leftDistance, rightDistance, initialPose} {
  for (size_t i = 0; i < 3; ++i) {
    m_q[i] = stateStdDevs[i] * stateStdDevs[i];
  }

  SetVisionMeasurementStdDevs(visionMeasurementStdDevs);
}

void DifferentialDrivePoseEstimator::SetVisionMeasurementStdDevs(
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

void DifferentialDrivePoseEstimator::ResetPosition(const Rotation2d& gyroAngle,
                                                   units::meter_t leftDistance,
                                                   units::meter_t rightDistance,
                                                   const Pose2d& pose) {
  // Reset state estimate and error covariance
  m_odometry.ResetPosition(gyroAngle, leftDistance, rightDistance, pose);
  m_poseBuffer.Clear();
}

Pose2d DifferentialDrivePoseEstimator::GetEstimatedPosition() const {
  return m_odometry.GetPose();
}

void DifferentialDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
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
      m_visionK *
      frc::Vectord<3>{twist.dx.value(), twist.dy.value(), twist.dtheta.value()};

  // Step 4: Convert back to Twist2d
  Twist2d scaledTwist{units::meter_t{k_times_twist(0)},
                      units::meter_t{k_times_twist(1)},
                      units::radian_t{k_times_twist(2)}};

  // Step 5: Apply scaled twist to the latest pose
  auto estimatedPose = GetEstimatedPosition().Exp(scaledTwist);

  // Step 6: Apply new pose to odometry
  m_odometry.ResetPosition(sample.value().gyroAngle, sample.value().leftDistance,
                           sample.value().rightDistance, estimatedPose);

  auto internal_buf = m_poseBuffer.GetInternalStructure();

  auto upper_bound = std::lower_bound(
    internal_buf.begin(), internal_buf.end(), timestamp,
    [](const auto& pair, auto t) { return t > pair.first; });

  for(auto entry = upper_bound; entry != internal_buf.end(); entry++) {
    UpdateWithTime(
      entry->first,
      entry->second.gyroAngle,
      entry->second.leftDistance,
      entry->second.rightDistance
    );
  }
}

Pose2d DifferentialDrivePoseEstimator::Update(const Rotation2d& gyroAngle,
                                              units::meter_t leftDistance,
                                              units::meter_t rightDistance) {
  return UpdateWithTime(units::microsecond_t(wpi::Now()), gyroAngle,
                        leftDistance, rightDistance);
}

Pose2d DifferentialDrivePoseEstimator::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& gyroAngle,
    units::meter_t leftDistance, units::meter_t rightDistance) {
  m_odometry.Update(gyroAngle, leftDistance, rightDistance);

  m_poseBuffer.AddSample(currentTime, {GetEstimatedPosition(), gyroAngle, leftDistance, rightDistance});

  return GetEstimatedPosition();
}
