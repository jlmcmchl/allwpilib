// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory/MotionProfile.h"
#include "frc/geometry/Rotation2d.h"

namespace frc {

/**
 * A specialized motion profile follower for rotational motion.
 *
 * This class extends the functionality of a standard motion profile follower
 * to handle the unique characteristics of rotational motion, including proper handling
 * of angle wrapping and shortest-path determination between two angles.
 *
 * @tparam P The type of motion profile to follow.
 */
template <typename P>
class RotationMotionProfileFollower {
 public:
  /**
   * Creates a new RotationMotionProfileFollower for the specified profile.
   *
   * @param profile The motion profile to follow.
   */
  explicit RotationMotionProfileFollower(P profile) : m_profile(profile) {}

  /**
   * Gets the current state of the follower.
   *
   * @return A copy of the current position and velocity state.
   */
  MotionProfile::State GetCurrentState() const {
    return MotionProfile::State(m_currentState.position, m_currentState.velocity);
  }

  /**
   * Gets the goal state of the follower.
   *
   * @return A copy of the goal position and velocity state.
   */
  MotionProfile::State GetGoalState() const {
    return MotionProfile::State(m_goalState.position, m_goalState.velocity);
  }

  /**
   * Sets the current state of the follower.
   *
   * The position is specified as a Rotation2d to properly handle angles, and the
   * internal state is normalized to find the shortest path to the goal.
   *
   * @param position The current angular position.
   * @param velocity The current angular velocity in radians per second.
   */
  void SetCurrentState(const Rotation2d& position, double velocity) {
    m_currentState.position = position.Radians();
    m_currentState.velocity = velocity;
    FixCurrent();
  }

  /**
   * Sets the goal state of the follower.
   *
   * The position is specified as a Rotation2d to properly handle angles, and the
   * internal state is normalized to find the shortest path to the goal.
   *
   * @param position The goal angular position.
   * @param velocity The goal angular velocity in radians per second.
   */
  void SetGoalState(const Rotation2d& position, double velocity) {
    m_goalState.position = position.Radians();
    m_goalState.velocity = velocity;
    FixCurrent();
  }

  /**
   * Updates the follower's state based on the elapsed time.
   *
   * This method advances the motion profile by the specified time step and updates
   * the internal current state.
   *
   * @param dt The elapsed time since the last update, in seconds.
   * @return The new state after the update.
   */
  MotionProfile::State Update(double dt) {
    auto state = m_profile.Calculate(dt, m_currentState, m_goalState);
    m_currentState.position = state.position;
    m_currentState.velocity = state.velocity;
    return state;
  }

  /**
   * Calculates the time remaining until the profile reaches the goal state.
   *
   * @return The time remaining in seconds.
   */
  double TimeRemaining() const {
    return m_profile.TimeRemaining(m_currentState, m_goalState);
  }

 private:
  /**
   * Adjusts the current position to ensure the shortest path is taken to the goal.
   *
   * This method handles angle wrapping by adding or subtracting 2π radians from
   * the current position to ensure the shortest angular path is followed.
   *
   * @return True if the current position was adjusted, false otherwise.
   */
  bool FixCurrent() {
    double currentTime = TimeRemaining();

    double adjustment = 0;
    if (m_goalState.position >= m_currentState.position) {
      adjustment = 2 * std::numbers::pi;
    } else {
      adjustment -= 2 * std::numbers::pi;
    }
    m_currentState.position += adjustment;

    if (TimeRemaining() < currentTime) {
      return true;
    }

    m_currentState.position -= adjustment;
    return false;
  }

  P m_profile;
  MotionProfile::State m_currentState;
  MotionProfile::State m_goalState;
};

}  // namespace frc 