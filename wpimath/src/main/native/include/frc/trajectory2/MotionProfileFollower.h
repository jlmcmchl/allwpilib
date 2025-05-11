// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory2/MotionProfile.h"

namespace frc {

/**
 * A follower for motion profiles that tracks the progress of a profile over time.
 *
 * This class maintains the current and goal states of a motion profile and provides methods
 * to update the profile state based on elapsed time. It's useful for implementing controllers
 * that need to follow a motion profile trajectory.
 *
 * @tparam P The type of motion profile to follow.
 */
template <class Distance>
class MotionProfileFollower {
 public:
  using State = typename MotionProfile<Distance>::State;

  /**
   * Creates a new MotionProfileFollower for the specified profile.
   *
   * @param profile The motion profile to follow.
   */
  MotionProfileFollower(MotionProfile<Distance> profile) : m_profile(profile) {}

  /**
   * Gets the current state of the follower.
   *
   * @return A copy of the current position and velocity state.
   */
  State GetCurrentState() const {
    return State(m_currentState.position, m_currentState.velocity);
  }

  /**
   * Gets the goal state of the follower.
   *
   * @return A copy of the goal position and velocity state.
   */
  State GetGoalState() const {
    return State(m_goalState.position, m_goalState.velocity);
  }

  /**
   * Sets the current state of the follower.
   *
   * @param current The new current state.
   */
  void SetCurrentState(const State& current) {
    m_currentState.position = current.position;
    m_currentState.velocity = current.velocity;
  }

  /**
   * Sets the goal state of the follower.
   *
   * @param goal The new goal state.
   */
  void SetGoalState(const State& goal) {
    m_goalState.position = goal.position;
    m_goalState.velocity = goal.velocity;
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
  State Update(double dt) {
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
  MotionProfile<Distance> m_profile;
  State m_currentState;
  State m_goalState;
};

}  // namespace frc 