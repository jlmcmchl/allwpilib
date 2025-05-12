// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "units/math.h"
#include "units/time.h"

namespace frc {

/**
 * Abstract base class for motion profiles that define position and velocity
 * trajectories over time.
 *
 * A motion profile generates a trajectory that smoothly transitions from one
 * state to another while respecting constraints such as maximum velocity and
 * acceleration. This class provides the foundation for various profile
 * implementations like trapezoidal and exponential profiles.
 *
 * Motion profiles are useful for:
 * - Generating smooth motions between setpoints
 * - Filtering control inputs to avoid jerky motions
 * - Respecting physical constraints of systems
 */
template <class Distance>
class MotionProfile {
 public:
  using Distance_t = units::unit_t<Distance>;
  using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;

  /**
   * Represents a state in a motion profile with position and velocity
   * components.
   */
  class State {
   public:
    /// The position at this state.
    Distance_t position{0};

    /// The velocity at this state.
    Velocity_t velocity{0};

    constexpr bool operator==(const State&) const = default;
  };

  /**
   * Abstract base class for curve segments in motion profiles.
   *
   * A curve represents a mathematical function defining motion between states,
   * with position and velocity characterized as functions of time.
   */
  class Curve {
   public:
    /**
     * Computes the distance required to reach the specified velocity.
     *
     * @param velocity The target velocity.
     * @return The distance required to reach the specified velocity.
     */
    virtual Distance_t ComputeDistanceFromVelocity(
        Velocity_t velocity) const = 0;

    /**
     * Computes the time required to reach the goal state.
     *
     * @param goal The target state.
     * @return The time required to reach the goal state.
     */
    virtual units::second_t TimeToState(const State& goal) const = 0;

    /**
     * Returns the state at the specified time along this curve.
     *
     * @param t The time since the start of the curve.
     * @return The state at time t.
     */
    State StateAtTime(units::second_t t) const {
      return State(ComputeDistanceFromTime(t), ComputeVelocityFromTime(t));
    }

    /**
     * Computes the velocity at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The velocity at time t.
     */
    virtual Velocity_t ComputeVelocityFromTime(units::second_t t) const = 0;

    /**
     * Computes the distance traveled at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The distance traveled at time t.
     */
    virtual Distance_t ComputeDistanceFromTime(units::second_t t) const = 0;

    /**
     * Calculates the velocity at which this curve intersects with another
     * curve.
     *
     * @param other The other curve to intersect with.
     * @return The velocity at the intersection point.
     */
    virtual Velocity_t IntersectionVelocity(const Curve& other) const = 0;

    /**
     * Finds the state at which this curve intersects with another curve.
     *
     * @param other The other curve to intersect with.
     * @return The state at the intersection point.
     */
    State Intersection(const Curve& other) const {
      Velocity_t intersectionVelocity = IntersectionVelocity(other);
      return State(ComputeDistanceFromVelocity(intersectionVelocity),
                   intersectionVelocity);
    }
  };

  /**
   * Abstract base class for motion constraints that determine the allowable
   * motion characteristics.
   *
   * @tparam T The type of curve that can be created from these constraints.
   */
  template <class Curve>
  class Constraints {
   public:
    /** The maximum allowable velocity for the motion profile. */
    Velocity_t maxVelocity{0};

    /**
     * Creates a curve passing through the given state in the specified
     * direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether the motion is in the negative (true) or positive
     * (false) direction.
     * @return A curve that passes through the state in the specified direction.
     */
    virtual Curve ThroughState(const State& state, bool direction) const = 0;

    /**
     * Sets the maximum velocity constraint.
     *
     * @param velocity The maximum velocity.
     * @return This object for method chaining.
     */
    Constraints& WithMaxVelocity(Velocity_t velocity) {
      maxVelocity = velocity;
      return *this;
    }
  };

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  virtual State Calculate(units::second_t t, const State& current,
                          const State& goal) const = 0;

  /**
   * Calculates the time remaining until the motion profile reaches the goal
   * state.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The time remaining until the goal state is reached.
   */
  virtual units::second_t TimeRemaining(const State& current,
                                        const State& goal) const = 0;

  /**
   * Determines whether the input direction should be flipped based on the
   * current and goal states.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (reversed), false
   * otherwise.
   */
  virtual bool ShouldFlipInput(const State& current,
                               const State& goal) const = 0;
};

}  // namespace frc
