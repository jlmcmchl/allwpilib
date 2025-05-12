// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

/**
 * A exponential curve-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on state-space model dynamics. To compute the reference
 * obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * ExponentialProfile.Constraints constraints =
 *   ExponentialProfile.Constraints.fromCharacteristics(kMaxV, kV, kA);
 * MotionProfile.State previousProfiledReference =
 *   new MotionProfile.State(initialReference, 0.0);
 * ExponentialProfile profile = constraints.asMotionProfile();
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
public class ExponentialProfile {
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
   * Constraints for an exponential profile.
   */
  public static class Constraints extends MotionProfile.Constraints {
    /** The maximum input value that can be applied to the system. */
    public final double maxInput;

    /** The state matrix coefficient for the system dynamics. */
    public final double A;

    /** The input matrix coefficient for the system dynamics. */
    public final double B;

    /**
     * Creates a new set of exponential profile constraints.
     *
     * @param maxInput The maximum input value that can be applied.
     * @param A The state matrix coefficient for the system dynamics.
     * @param B The input matrix coefficient for the system dynamics.
     */
    private Constraints(double maxInput, double A, double B) {
      this.maxInput = maxInput;
      this.A = A;
      this.B = B;
    }

    /**
     * Creates constraints from the physical system characteristics.
     *
     * @param maxInput The maximum input value.
     * @param kV The velocity gain (voltage per velocity).
     * @param kA The acceleration gain (voltage per acceleration).
     * @return A new Constraints object.
     */
    public static Constraints fromCharacteristics(double maxInput, double kV, double kA) {
      return new Constraints(maxInput, -kV / kA, 1.0 / kA);
    }

    /**
     * Creates constraints from state-space model parameters.
     *
     * @param maxInput The maximum input value.
     * @param A The state matrix coefficient for the system dynamics.
     * @param B The input matrix coefficient for the system dynamics.
     * @return A new Constraints object.
     */
    public static Constraints fromStateSpace(double maxInput, double A, double B) {
      return new Constraints(maxInput, A, B);
    }

    /**
     * Creates constraints from boundary conditions of the system.
     *
     * @param steadyStateVelocity The velocity at steady state with a unit input.
     * @param standstillAcceleration The acceleration at standstill with a unit input.
     * @return A new Constraints object.
     */
    public static Constraints fromBoundaryConditions(
        double steadyStateVelocity, double standstillAcceleration) {
      return fromCharacteristics(1, 1 / steadyStateVelocity, 1 / standstillAcceleration);
    }

    /**
     * Creates a curve passing through the given state in the specified direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether to move in the positive (false) or negative (true) direction.
     * @return A new curve passing through the given state.
     */
    @Override
    public Curve throughState(State state, boolean direction) {
      return new Curve(this, state, direction ? -maxInput : maxInput);
    }

    /**
     * Calculates the maximum achievable velocity for a given acceleration.
     *
     * @param acceleration The acceleration value to use.
     * @return The maximum achievable velocity.
     */
    public double MaxAchievableVelocity(double acceleration) {
      return (acceleration - B * maxInput) / A;
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
   * A curve segment of an exponential profile.
   */
  public static class Curve extends MotionProfile.Curve {
    private final State m_initialState;
    private final Constraints m_constraints;
    private final double m_input;

    /**
     * Creates a new exponential curve.
     *
     * @param constraints The constraints for the curve.
     * @param state The initial state for the curve.
     * @param input The input value to apply.
     */
    public Curve(Constraints constraints, State state, double input) {
      m_constraints = constraints;
      m_initialState = state;
      m_input = input;
    }

    /**
     * Computes the distance necessary to reach the specified velocity.
     *
     * @param velocity The target velocity.
     * @return The distance required to reach the specified velocity.
     */
    @Override
    public double computeDistanceFromVelocity(double velocity) {
      var A = m_constraints.A;
      var B = m_constraints.B;
      var u = m_input;

      return m_initialState.position
          + (velocity - m_initialState.velocity) / A
          - B
              * u
              / (A * A)
              * Math.log((A * velocity + B * u) / (A * m_initialState.velocity + B * u));
    }

    /**
     * Calculates the time needed to reach the goal state.
     *
     * @param goal The target state.
     * @return The time required to reach the goal state.
     */
    @Override
    public double timeToState(State goal) {
      var steadyStateVelocity =
          m_input >= 0
              ? m_constraints.MaxAchievableVelocity(0)
              : -m_constraints.MaxAchievableVelocity(0);
      var initial_error = m_initialState.velocity - steadyStateVelocity;
      var error = goal.velocity - steadyStateVelocity;

      if (Math.abs(error) < 1e-9 * m_constraints.MaxAchievableVelocity(0)) {
        // if we're near instability, get pretty close and use an approximation
        var nearVelocity =
            goal.velocity
                + Math.signum(initial_error) * 1e-9 * m_constraints.MaxAchievableVelocity(0);
        var nearPosition = computeDistanceFromVelocity(nearVelocity);
        var timeNearby = timeToState(new State(nearPosition, nearVelocity));

        var remainder = (goal.position - nearPosition) / steadyStateVelocity;

        return timeNearby + remainder;
      }

      var A = m_constraints.A;
      var B = m_constraints.B;
      var u = m_input;

      return Math.log((A * goal.velocity + B * u) / (A * m_initialState.velocity + B * u)) / A;
    }

    /**
     * Computes the velocity at a specific time.
     *
     * @param t The time since the start of the curve.
     * @return The velocity at time t.
     */
    @Override
    public double computeVelocityFromTime(double t) {
      var A = m_constraints.A;
      var B = m_constraints.B;
      var u = m_input;

      return (m_initialState.velocity + B * u / A) * Math.exp(A * t) - B * u / A;
    }

    /**
     * Computes the distance traveled at a specific time.
     *
     * @param t The time since the start of the curve.
     * @return The distance traveled at time t.
     */
    @Override
    public double computeDistanceFromTime(double t) {
      var A = m_constraints.A;
      var B = m_constraints.B;
      var u = m_input;

      return m_initialState.position
          + (-B * u * t + (m_initialState.velocity + B * u / A) * (Math.exp(A * t) - 1)) / A;
    }

    /**
     * Calculates the velocity at which this curve intersects with another curve.
     *
     * @param other The other curve to find intersection with.
     * @return The velocity at the intersection point, or 0 if no intersection exists or numerical issues occur.
     */
    @Override
    public double intersectionVelocity(MotionProfile.Curve other) {
      System.out.printf("intersectionVelocity: %s%n", other instanceof ExponentialProfile.Curve);
      if (other instanceof ExponentialProfile.Curve) {
        var otherCurve = (ExponentialProfile.Curve) other;
        var A = m_constraints.A;
        var B = m_constraints.B;
        var u = m_input;

        var U_dir = Math.signum(u);

        var position_delta = otherCurve.m_initialState.position - m_initialState.position;
        var velocity_delta = otherCurve.m_initialState.velocity - m_initialState.velocity;

        var scalar =
            (A * m_initialState.velocity + B * u)
                * (A * otherCurve.m_initialState.velocity - B * u);
        var power = -A / B / u * (A * position_delta - velocity_delta);

        var a = -A * A;
        var c = (B * B) * (u * u) + scalar * Math.exp(power);

        if (-1e-9 < c && c < 0) {
          // Numerical stability issue - the heuristic gets it right but c is around -1e-13
          return 0;
        }

        return U_dir * Math.sqrt(-c / a);
      }
      return 0;
    }
  }
}
