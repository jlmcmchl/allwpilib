// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory2/TrapezoidProfile.h"

namespace frc {

template <class Distance>
typename TrapezoidProfile<Distance>::Distance_t
TrapezoidProfile<Distance>::Curve::ComputeDistanceFromVelocity(
    Velocity_t velocity) const {
  // TODO: Implement distance calculation based on velocity
  return Distance_t{0};
}

template <class Distance>
units::second_t TrapezoidProfile<Distance>::Curve::TimeToState(
    const typename MotionProfile<Distance>::State& goal) const {
  // TODO: Implement time calculation to reach goal state
  return units::second_t{0};
}

template <class Distance>
typename TrapezoidProfile<Distance>::Velocity_t
TrapezoidProfile<Distance>::Curve::ComputeVelocityFromTime(
    units::second_t t) const {
  // TODO: Implement velocity calculation based on time
  return Velocity_t{0};
}

template <class Distance>
typename TrapezoidProfile<Distance>::Distance_t
TrapezoidProfile<Distance>::Curve::ComputeDistanceFromTime(
    units::second_t t) const {
  // TODO: Implement distance calculation based on time
  return Distance_t{0};
}

template <class Distance>
typename TrapezoidProfile<Distance>::Velocity_t
TrapezoidProfile<Distance>::Curve::IntersectionVelocity(
    const typename MotionProfile<Distance>::Curve& other) const {
  // TODO: Implement intersection velocity calculation
  return Velocity_t{0};
}

}  // namespace frc
