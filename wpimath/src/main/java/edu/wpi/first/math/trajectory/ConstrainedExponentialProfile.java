// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

public class ConstrainedExponentialProfile {
  public static FullStateMotionProfile FullState(Constraints constraints) {
    return new FullStateMotionProfile(constraints, constraints);
  }

  public static FullStateMotionProfile FullState(
      Constraints forwardConstraints, Constraints reverseConstraints) {
    return new FullStateMotionProfile(forwardConstraints, reverseConstraints);
  }

  public static VelocityMotionProfile VelocityOnly(Constraints constraints) {
    return new VelocityMotionProfile(constraints, constraints);
  }

  public static VelocityMotionProfile VelocityOnly(
      Constraints forwardConstraints, Constraints reverseConstraints) {
    return new VelocityMotionProfile(forwardConstraints, reverseConstraints);
  }

  public static class Constraints extends MotionProfile.Constraints {
    private final TrapezoidProfile.Constraints m_trapezoid;
    private final ExponentialProfile.Constraints m_exponential;

    private Constraints(
        TrapezoidProfile.Constraints trapezoid, ExponentialProfile.Constraints exponential) {
      m_trapezoid = trapezoid;
      m_exponential = exponential;
    }

    public Curve throughState(State state, boolean direction) {
      var switchingVelocity = SwitchingVelocity();
      double switchingPosition;

      if (state.velocity <= SwitchingVelocity()) {
        switchingPosition =
            m_trapezoid
                .throughState(state, direction)
                .computeDistanceFromVelocity(switchingVelocity);
      } else {
        switchingPosition =
            m_exponential
                .throughState(state, direction)
                .computeDistanceFromVelocity(switchingVelocity);
      }

      return new Curve(this, new State(switchingVelocity, switchingPosition), state, direction);
    }

    public double SwitchingVelocity() {
      return m_exponential.MaxAchievableVelocity(m_trapezoid.maxAcceleration);
    }

    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }
  }

  public static class Curve extends MotionProfile.Curve {
    private final Constraints m_constraints;
    private final State m_initialState;
    private final State m_switchingState;
    private final TrapezoidProfile.Curve m_trapezoid;
    private final ExponentialProfile.Curve m_exponential;
    private final double m_timeToSwitchingState;
    private final boolean m_direction;

    private Curve(
        Constraints constraints, State switchingState, State initialState, boolean direction) {
      m_constraints = constraints;
      m_trapezoid = constraints.m_trapezoid.throughState(switchingState, direction);
      m_exponential = constraints.m_exponential.throughState(switchingState, direction);
      m_initialState = initialState;
      m_switchingState = switchingState;
      m_direction = direction;

      if (m_initialState.velocity > m_switchingState.velocity) {
        m_timeToSwitchingState = -m_exponential.timeToState(initialState);
      } else {
        m_timeToSwitchingState = -m_trapezoid.timeToState(initialState);
      }
    }

    @Override
    public double computeDistanceFromVelocity(double velocity) {
      if (Math.abs(velocity - m_switchingState.velocity) > 0) {
        return m_exponential.computeDistanceFromVelocity(velocity);
      }

      return m_trapezoid.computeDistanceFromVelocity(velocity);
    }

    @Override
    public double timeToState(State goal) {
      if (Math.abs(goal.velocity - m_switchingState.velocity) > 0) {
        return m_exponential.timeToState(goal) + m_timeToSwitchingState;
      }

      return m_trapezoid.timeToState(goal) + m_timeToSwitchingState;
    }

    @Override
    public double computeVelocityFromTime(double t) {
      if (t > m_timeToSwitchingState) {
        return m_exponential.computeVelocityFromTime(t - m_timeToSwitchingState);
      }

      return m_trapezoid.computeVelocityFromTime(t);
    }

    @Override
    public double computeDistanceFromTime(double t) {
      if (t > m_timeToSwitchingState) {
        return m_exponential.computeDistanceFromTime(t - m_timeToSwitchingState);
      }

      return m_trapezoid.computeDistanceFromTime(t);
    }

    @Override
    public double intersectionVelocity(MotionProfile.Curve second) {
      return 0;
    }
  }
}
