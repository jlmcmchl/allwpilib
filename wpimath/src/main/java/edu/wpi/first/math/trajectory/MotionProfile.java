// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import java.util.Objects;

/**
 * Abstract base class for motion profiles that define position and velocity trajectories over time.
 *
 * <p>A motion profile generates a trajectory that smoothly transitions from one state to another
 * while respecting constraints such as maximum velocity and acceleration. This class provides
 * the foundation for various profile implementations like trapezoidal and exponential profiles.
 *
 * <p>Motion profiles are useful for:
 * <ul>
 *   <li>Generating smooth motions between setpoints</li>
 *   <li>Filtering control inputs to avoid jerky motions</li>
 *   <li>Respecting physical constraints of systems</li>
 * </ul>
 */
public abstract class MotionProfile {
  /**
   * Creates a FullStateMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  public static FullStateMotionProfile FullState(Constraints constraints) {
    return new FullStateMotionProfile(constraints, constraints);
  }

  /**
   * Creates a FullStateMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  public static FullStateMotionProfile FullState(
      Constraints forwardConstraints, Constraints reverseConstraints) {
    return new FullStateMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Creates a VelocityMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  public static VelocityMotionProfile VelocityOnly(Constraints constraints) {
    return new VelocityMotionProfile(constraints, constraints);
  }

  /**
   * Creates a VelocityMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  public static VelocityMotionProfile VelocityOnly(
      Constraints forwardConstraints, Constraints reverseConstraints) {
    return new VelocityMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Represents a state in a motion profile with position and velocity components.
   */
  public static class State {
    /** The position component of the state. */
    public double position;

    /** The velocity component of the state. */
    public double velocity;

    /**
     * Creates a new State with position and velocity initialized to zero.
     */
    public State() {}

    /**
     * Creates a new State with the specified position and velocity.
     *
     * @param position The position component.
     * @param velocity The velocity component.
     */
    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    /**
     * Checks if this State is equal to another object.
     *
     * @param other The object to compare with this State.
     * @return True if the other object is a State with the same position and velocity.
     */
    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        return this.position == rhs.position && this.velocity == rhs.velocity;
      } else {
        return false;
      }
    }

    /**
     * Generates a hash code for this State.
     *
     * @return A hash code based on position and velocity.
     */
    @Override
    public int hashCode() {
      return Objects.hash(position, velocity);
    }

    /**
     * Returns a string representation of this State.
     *
     * @return A string in the format "State(position, velocity)".
     */
    @Override
    public String toString() {
      return String.format("State(%s, %s)", position, velocity);
    }
  }

  /**
   * Abstract base class for motion constraints that determine the allowable motion characteristics.
   *
   * @param <T> The type of curve that can be created from these constraints.
   */
  public abstract static class Constraints {
    /** The maximum allowable velocity for the motion profile. Default is unlimited. */
    public double maxVelocity = Double.MAX_VALUE;

    /**
     * Creates a curve passing through the given state in the specified direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether the motion is in the negative (true) or positive (false) direction.
     * @return A curve that passes through the state in the specified direction.
     */
    public abstract Curve throughState(State state, boolean direction);

    /**
     * Sets the maximum velocity constraint.
     *
     * @param velocity The maximum velocity.
     * @return This object for method chaining.
     */
    public Constraints withMaxVelocity(double velocity) {
      this.maxVelocity = velocity;

      return this;
    }
  }

  /**
   * Abstract base class for curve segments in motion profiles.
   *
   * <p>A curve represents a mathematical function defining motion between states,
   * with position and velocity characterized as functions of time.
   */
  public abstract static class Curve {
    /**
     * Computes the distance required to reach the specified velocity.
     *
     * @param velocity The target velocity.
     * @return The distance required to reach the specified velocity.
     */
    public abstract double computeDistanceFromVelocity(double velocity);

    /**
     * Computes the time required to reach the goal state.
     *
     * @param goal The target state.
     * @return The time required to reach the goal state.
     */
    public abstract double timeToState(State goal);

    /**
     * Returns the state at the specified time along this curve.
     *
     * @param t The time since the start of the curve.
     * @return The state at time t.
     */
    public State stateAtTime(double t) {
      return new State(computeDistanceFromTime(t), computeVelocityFromTime(t));
    }

    /**
     * Computes the velocity at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The velocity at time t.
     */
    public abstract double computeVelocityFromTime(double t);

    /**
     * Computes the distance traveled at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The distance traveled at time t.
     */
    public abstract double computeDistanceFromTime(double t);

    /**
     * Calculates the velocity at which this curve intersects with another curve.
     *
     * @param other The other curve to intersect with.
     * @return The velocity at the intersection point.
     */
    public abstract double intersectionVelocity(Curve other);

    /**
     * Finds the state at which this curve intersects with another curve.
     *
     * @param other The other curve to intersect with.
     * @return The state at the intersection point.
     */
    public State intersection(Curve other) {
      var intersectionVelocity = intersectionVelocity(other);
      return new State(computeDistanceFromVelocity(intersectionVelocity), intersectionVelocity);
    }
  }

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  public abstract State calculate(double t, State current, State goal);

  /**
   * Calculates the time remaining until the motion profile reaches the goal state.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The time remaining until the goal state is reached.
   */
  public abstract double timeRemaining(State current, State goal);

  /**
   * Determines whether the input direction should be flipped based on the current and goal states.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (reversed), false otherwise.
   */
  public abstract boolean shouldFlipInput(State current, State goal);

  /**
   * Creates a new motion profile follower for this profile.
   *
   * @return A motion profile follower for this profile.
   */
  public MotionProfileFollower follower() {
    return new MotionProfileFollower(this);
  }

  /**
   * Creates a new rotation motion profile follower for this profile.
   *
   * @return A rotation motion profile follower for this profile.
   */
  public RotationMotionProfileFollower rotationFollower() {
    return new RotationMotionProfileFollower(this);
  }
}
