// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>

#include "frc/trajectory2/MotionProfile.h"
#include "units/math.h"

namespace frc {

/**
 * A motion profile that controls both position and velocity states, with full
 * state feedback.
 *
 * This class generates motion profiles that respect both position and velocity
 * constraints, creating smooth transitions between states. It can use different
 * constraints for forward and reverse motion, allowing for asymmetric
 * acceleration and deceleration profiles.
 *
 * The profile automatically determines the optimal direction of travel and
 * handles cases where velocity limits are reached.
 */
template <class Distance, class ForwardCurve, class ReverseCurve>
class FullStateMotionProfile : public MotionProfile<Distance> {
 public:
  using State = typename MotionProfile<Distance>::State;
  using ForwardConstraints =
      typename MotionProfile<Distance>::template Constraints<ForwardCurve>;
  using ReverseConstraints =
      typename MotionProfile<Distance>::template Constraints<ReverseCurve>;
  /**
   * Creates a new FullStateMotionProfile with the given constraints.
   *
   * @param forwardConstraints The constraints to use for forward motion.
   * @param reverseConstraints The constraints to use for reverse motion.
   */
  FullStateMotionProfile(const ForwardConstraints& forwardConstraints,
                         const ReverseConstraints& reverseConstraints)
      : m_forwardConstraints(forwardConstraints),
        m_reverseConstraints(reverseConstraints),
        m_maxVelocity(std::min(forwardConstraints.maxVelocity,
                               reverseConstraints.maxVelocity)) {}

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * This method determines the optimal trajectory between the current state and
   * goal state, respecting all motion constraints, and returns the state at the
   * specified time along that trajectory.
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
  State Calculate(units::second_t t, const State& current,
                  const State& goal) const {
    bool direction = ShouldFlipInput(current, goal);

    auto forwardCurve = m_forwardConstraints.ThroughState(current, direction);
    auto reverseCurve = m_reverseConstraints.ThroughState(goal, !direction);

    auto intersection = forwardCurve.Intersection(reverseCurve);

    auto maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    if (units::math::abs(current.velocity) > m_maxVelocity) {
      forwardCurve = m_reverseConstraints.ThroughState(current, !direction);
      reverseCurve = m_forwardConstraints.ThroughState(current, direction);
    } else {
      forwardCurve = m_forwardConstraints.ThroughState(current, direction);
      reverseCurve =
          m_reverseConstraints.ThroughState(intersection, !direction);
    }

    if (units::math::abs(intersection.velocity) < m_maxVelocity) {
      auto timeAccelerating = forwardCurve.TimeToState(intersection);

      if (timeAccelerating >= t) {
        return forwardCurve.StateAtTime(t);
      }

      auto timeDecelerating = reverseCurve.TimeToState(goal);

      if (timeAccelerating + timeDecelerating >= t) {
        return reverseCurve.StateAtTime(t - timeAccelerating);
      }

      return goal;
    }

    State forwardIntersectionWithMaxVelocity(
        forwardCurve.ComputeDistanceFromVelocity(maxVelocity), maxVelocity);

    auto timeAccelerating =
        forwardCurve.TimeToState(forwardIntersectionWithMaxVelocity);

    if (timeAccelerating >= t) {
      return forwardCurve.StateAtTime(t);
    }

    State reverseIntersectionWithMaxVelocity(
        reverseCurve.ComputeDistanceFromVelocity(maxVelocity), maxVelocity);

    reverseCurve = m_reverseConstraints.ThroughState(
        reverseIntersectionWithMaxVelocity, !direction);

    auto timeAtMaxVelocity = (reverseIntersectionWithMaxVelocity.position -
                              forwardIntersectionWithMaxVelocity.position) /
                             maxVelocity;

    if (timeAccelerating + timeAtMaxVelocity >= t) {
      return State(forwardIntersectionWithMaxVelocity.position +
                       maxVelocity * (t - timeAccelerating),
                   maxVelocity);
    }

    auto timeDecelerating = reverseCurve.TimeToState(goal);

    if (timeAccelerating + timeAtMaxVelocity + timeDecelerating >= t) {
      return reverseCurve.StateAtTime(t -
                                      (timeAccelerating + timeAtMaxVelocity));
    }

    return goal;
  }

  /**
   * Calculates the time remaining until the motion profile reaches the goal
   * state.
   *
   * This method determines the total time required to move from the current
   * state to the goal state along the optimal trajectory, respecting all motion
   * constraints.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The time remaining until the goal state is reached.
   */
  units::second_t TimeRemaining(const State& current, const State& goal) const {
    bool direction = ShouldFlipInput(current, goal);

    auto forwardCurve m_forwardConstraints.ThroughState(current, direction);
    auto reverseCurve = m_reverseConstraints.ThroughState(goal, !direction);

    auto intersection = forwardCurve.Intersection(reverseCurve);

    if (units::math::abs(current.velocity) > m_maxVelocity) {
      forwardCurve = m_forwardConstraints.ThroughState(current, !direction);
    } else {
      forwardCurve = m_forwardConstraints.ThroughState(current, direction);
    }

    reverseCurve = m_reverseConstraints.ThroughState(intersection, !direction);

    if (units::math::abs(intersection.velocity) < m_maxVelocity &&
        units::math::abs(current.velocity) < m_maxVelocity) {
      units::second_t timeAccelerating = forwardCurve.TimeToState(intersection);
      units::second_t timeDecelerating = reverseCurve.TimeToState(goal);

      return timeAccelerating + timeDecelerating;
    }

    auto maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    State forwardIntersectionWithMaxVelocity(
        forwardCurve.ComputeDistanceFromVelocity(maxVelocity), maxVelocity);

    units::second_t timeAccelerating =
        forwardCurve.TimeToState(forwardIntersectionWithMaxVelocity);

    State reverseIntersectionWithMaxVelocity(
        reverseCurve.ComputeDistanceFromVelocity(maxVelocity), maxVelocity);

    reverseCurve = m_reverseConstraints.ThroughState(
        reverseIntersectionWithMaxVelocity, !direction);

    units::second_t timeAtMaxVelocity =
        (reverseIntersectionWithMaxVelocity.position -
         forwardIntersectionWithMaxVelocity.position) /
        maxVelocity;

    units::second_t timeDecelerating = reverseCurve.TimeToState(goal);

    return timeAccelerating + timeAtMaxVelocity + timeDecelerating;
  }

  /**
   * Determines whether the input direction should be flipped based on the
   * current and goal states.
   *
   * This method examines the current position, velocity, and goal states to
   * determine the optimal direction for motion. It handles cases where direct
   * paths may not be feasible due to velocity constraints or when reversing
   * direction is more efficient.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (reversed), false
   * otherwise.
   */
  bool ShouldFlipInput(const State& current, const State& goal) const {
    auto xf = goal.position;
    auto v0 = current.velocity;
    auto vf = goal.velocity;

    auto x_forward = m_forwardConstraints.ThroughState(current, false)
                         .ComputeDistanceFromVelocity(vf);
    auto x_reverse = m_forwardConstraints.ThroughState(current, true)
                         .ComputeDistanceFromVelocity(vf);

    if (units::math::is_nan(x_forward)) {
      return xf < x_reverse;
    }

    if (units::math::is_nan(x_reverse)) {
      return xf < x_forward;
    }

    bool a = v0 >= typename MotionProfile<Distance>::Velocity_t(0);
    bool b = vf >= typename MotionProfile<Distance>::Velocity_t(0);
    bool c = xf >= x_forward;
    bool d = xf >= x_reverse;

    return (a && !d) || (b && !c) || (!c && !d);
  }

 private:
  const ForwardConstraints& m_forwardConstraints;
  const ReverseConstraints& m_reverseConstraints;
  const MotionProfile<Distance>::Velocity_t m_maxVelocity;
};

}  // namespace frc
