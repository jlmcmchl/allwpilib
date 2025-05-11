// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory2/MotionProfile.h"

namespace frc {

/**
 * A motion profile that controls both position and velocity states, with full state feedback.
 * 
 * This class generates motion profiles that respect both position and velocity constraints,
 * creating smooth transitions between states. It can use different constraints for forward and
 * reverse motion, allowing for asymmetric acceleration and deceleration profiles.
 * 
 * The profile automatically determines the optimal direction of travel and handles
 * cases where velocity limits are reached.
 */
template <class Distance>
class FullStateMotionProfile : public MotionProfile<Distance> {
 public:
  using State = typename MotionProfile<Distance>::State;
  using Constraints = typename MotionProfile<Distance>::Constraints;
  /**
   * Creates a new FullStateMotionProfile with the given constraints.
   *
   * @param forwardConstraints The constraints to use for forward motion.
   * @param reverseConstraints The constraints to use for reverse motion.
   */
  FullStateMotionProfile(const Constraints& forwardConstraints,
                        const Constraints& reverseConstraints)
      : m_forwardConstraints(forwardConstraints),
        m_reverseConstraints(reverseConstraints),
        m_maxVelocity(std::min(forwardConstraints.maxVelocity, reverseConstraints.maxVelocity)) {}

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * This method determines the optimal trajectory between the current state and goal state,
   * respecting all motion constraints, and returns the state at the specified time along
   * that trajectory.
   *
   * The generated trajectory consists of up to three segments:
   * 1. An acceleration segment using the forward constraints
   * 2. An optional constant velocity segment at maximum velocity
   * 3. A deceleration segment using the reverse constraints
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  State Calculate(double t, const State& current,
                               const State& goal) const;

  /**
   * Calculates the time remaining until the motion profile reaches the goal state.
   *
   * This method determines the total time required to move from the current state to the
   * goal state along the optimal trajectory, respecting all motion constraints.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The time remaining until the goal state is reached.
   */
  units::second_t TimeRemaining(const State& current,
                      const State& goal) const;

  /**
   * Determines whether the input direction should be flipped based on the current and goal states.
   *
   * This method examines the current position, velocity, and goal states to determine the
   * optimal direction for motion. It handles cases where direct paths may not be feasible
   * due to velocity constraints or when reversing direction is more efficient.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (reversed), false otherwise.
   */
  bool ShouldFlipInput(const State& current,
                      const State& goal) const;

 private:
  const Constraints& m_forwardConstraints;
  const Constraints& m_reverseConstraints;
  const double m_maxVelocity;
};

}  // namespace frc 