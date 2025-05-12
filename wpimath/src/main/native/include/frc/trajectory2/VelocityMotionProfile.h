// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory2/MotionProfile.h"

namespace frc {

/**
 * A motion profile that focuses only on velocity constraints without full
 * position control.
 *
 * Unlike a full-state motion profile, this profile only attempts to reach the
 * goal velocity without ensuring the exact goal position is reached. It's
 * useful for applications where precise position control is less important than
 * velocity tracking.
 */
template <class Distance, class ForwardCurve, class ReverseCurve>
class VelocityMotionProfile : public MotionProfile<Distance> {
 public:
  using State = MotionProfile<Distance>::State;
  using ForwardConstraints =
      MotionProfile<Distance>::template Constraints<ForwardCurve>;
  using ReverseConstraints =
      MotionProfile<Distance>::template Constraints<ReverseCurve>;
  /**
   * Creates a new VelocityMotionProfile with the given constraints.
   *
   * @param forwardConstraints The constraints to use for forward motion
   * (accelerating).
   * @param reverseConstraints The constraints to use for reverse motion
   * (decelerating).
   */
  VelocityMotionProfile(const ForwardConstraints& forwardConstraints,
                        const ReverseConstraints& reverseConstraints);

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * This method determines the optimal trajectory to reach the goal velocity,
   * respecting all motion constraints, and returns the state at the specified
   * time along that trajectory.
   *
   * Once the goal velocity is reached, the position continues to increase at
   * that velocity.
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  State Calculate(double t, const State& current,
                  const State& goal) const override;

  /**
   * Calculates the time remaining until the motion profile reaches the goal
   * velocity.
   *
   * This method determines the total time required to transition from the
   * current velocity to the goal velocity, respecting all motion constraints.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (only velocity is considered).
   * @return The time remaining until the goal velocity is reached.
   */
  units::second_t TimeRemaining(const State& current,
                                const State& goal) const override;

  /**
   * Determines whether the input direction should be flipped based on the
   * current and goal velocities.
   *
   * For a velocity profile, the direction only depends on whether we need to
   * increase or decrease velocity to reach the goal.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (when we need to
   * decrease velocity), false otherwise.
   */
  bool ShouldFlipInput(const State& current, const State& goal) const override;

 private:
  const ForwardConstraints& m_forwardConstraints;
  const ReverseConstraints& m_reverseConstraints;
};

}  // namespace frc
