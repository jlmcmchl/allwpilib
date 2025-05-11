// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.trajectory.MotionProfile.State;

/**
 * A trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
 * reference obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * TrapezoidCurve.Constraints constraints =
 *   new TrapezoidCurve.Constraints(kMaxV, kMaxA);
 * MotionProfile.State previousProfiledReference =
 *   new MotionProfile.State(initialReference, 0.0);
 * TrapezoidProfile profile = constraints.asMotionProfile();
 * </code></pre>
 *
 * <p>Run on update:
 *
 * <pre><code>
 * previousProfiledReference =
 * profile.calculate(timeSincePreviousUpdate, previousProfiledReference, unprofiledReference);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `timeRemaining()`.
 */
public class TrapezoidProfile {
  /**
   * Creates a FullStateMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  public static FullStateMotionProfile FullState(Constraints constraints) {
    MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);

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
    MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);

    return new FullStateMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Creates a VelocityMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  public static VelocityMotionProfile VelocityOnly(Constraints constraints) {
    MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);

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
    MathSharedStore.reportUsage(MathUsageId.kTrajectory_TrapezoidProfile, 1);

    return new VelocityMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Constraints for a trapezoidal motion profile.
   */
  public static class Constraints extends MotionProfile.Constraints {
    /** The maximum acceleration for the profile. */
    public final double maxAcceleration;

    /**
     * Creates a new set of trapezoidal profile constraints with only acceleration limits.
     *
     * @param acceleration The maximum acceleration.
     */
    public Constraints(double acceleration) {
      this.maxAcceleration = acceleration;
    }

    /**
     * Creates a new set of trapezoidal profile constraints with both velocity and acceleration limits.
     *
     * @param maxVelocity The maximum velocity.
     * @param acceleration The maximum acceleration.
     */
    public Constraints(double maxVelocity, double acceleration) {
      this.maxAcceleration = acceleration;

      withMaxVelocity(maxVelocity);
    }

    /**
     * Creates a curve passing through the given state in the specified direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether to move in the negative (true) or positive (false) direction.
     * @return A new curve passing through the given state.
     */
    @Override
    public Curve throughState(State state, boolean direction) {
      return new Curve(this, state, direction ? -maxAcceleration : maxAcceleration);
    }

    /**
     * Sets the maximum velocity constraint.
     *
     * @param velocity The maximum velocity.
     * @return This object for method chaining.
     */
    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }
  }

  /**
   * A curve segment of a trapezoidal profile.
   */
  public static class Curve extends MotionProfile.Curve {
    private final Constraints m_constraints;
    private final State m_initialState;
    private final double m_acceleration;

    /**
     * Creates a new trapezoidal curve.
     *
     * @param constraints The constraints for the curve.
     * @param state The initial state for the curve.
     * @param acceleration The acceleration to apply.
     */
    public Curve(Constraints constraints, State state, double acceleration) {
      m_constraints = constraints;
      m_initialState = state;
      m_acceleration = acceleration;
    }

    /**
     * Computes the distance required to reach the specified velocity.
     *
     * @param velocity The target velocity.
     * @return The distance required to reach the specified velocity.
     */
    @Override
    public double computeDistanceFromVelocity(double velocity) {
      return m_initialState.position
          + (velocity * velocity - m_initialState.velocity * m_initialState.velocity)
              / (2 * m_acceleration);
    }

    /**
     * Computes the time required to reach the goal state.
     *
     * @param goal The target state.
     * @return The time required to reach the goal state.
     */
    @Override
    public double timeToState(State goal) {
      return (goal.velocity - m_initialState.velocity) / m_acceleration;
    }

    /**
     * Computes the velocity at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The velocity at time t.
     */
    @Override
    public double computeVelocityFromTime(double t) {
      return m_initialState.velocity + m_acceleration * t;
    }

    /**
     * Computes the distance traveled at the specified time.
     *
     * @param t The time since the start of the curve.
     * @return The distance traveled at time t.
     */
    @Override
    public double computeDistanceFromTime(double t) {
      return m_initialState.position + m_initialState.velocity * t + m_acceleration * t * t / 2;
    }

    /**
     * Calculates the velocity at which this curve intersects with another curve.
     *
     * @param other The other curve to intersect with.
     * @return The velocity at the intersection point, or 0 if no intersection exists.
     */
    @Override
    public double intersectionVelocity(MotionProfile.Curve other) {
      if (other instanceof TrapezoidProfile.Curve) {
        var otherCurve = (TrapezoidProfile.Curve) other;
        double positionDifference = otherCurve.m_initialState.position - m_initialState.position;
        double velocitySum =
            m_initialState.velocity * m_initialState.velocity
                + otherCurve.m_initialState.velocity * otherCurve.m_initialState.velocity;

        double sqrtTerm = Math.sqrt(m_acceleration * positionDifference + velocitySum / 2);

        // Calculate the sign based on the acceleration direction
        double sign = Math.signum(m_acceleration);

        return sign * sqrtTerm;
      }
      return 0;
    }
  }
}
