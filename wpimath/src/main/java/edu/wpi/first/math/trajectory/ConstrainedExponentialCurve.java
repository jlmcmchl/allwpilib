package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

public class ConstrainedExponentialCurve extends MotionCurve<ConstrainedExponentialCurve> {
  private final Constraints m_constraints;
  private final State m_initialState;
  private final State m_switchingState;
  private final TrapezoidCurve m_trapezoid;
  private final ExponentialCurve m_exponential;
  private final double m_timeToSwitchingState;
  private final boolean m_direction;

  private ConstrainedExponentialCurve(
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

  public static class Constraints
      extends edu.wpi.first.math.trajectory.MotionCurve.Constraints<ConstrainedExponentialCurve> {
    private final TrapezoidCurve.Constraints m_trapezoid;
    private final ExponentialCurve.Constraints m_exponential;

    private Constraints(
        TrapezoidCurve.Constraints trapezoid, ExponentialCurve.Constraints exponential) {
      m_trapezoid = trapezoid;
      m_exponential = exponential;
    }

    public ConstrainedExponentialCurve throughState(State state, boolean direction) {
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

      return new ConstrainedExponentialCurve(
          this, new State(switchingVelocity, switchingPosition), state, direction);
    }

    public double SwitchingVelocity() {
      return m_exponential.MaxAchievableVelocity(m_trapezoid.maxAcceleration);
    }

    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }

    @Override
    public ConstrainedExponentialProfile asMotionProfile() {
      return new ConstrainedExponentialProfile(this);
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
  public double intersectionVelocity(ConstrainedExponentialCurve second) {
    var v_last = m_switchingState.velocity;
    var x_last =
        second.computeDistanceFromVelocity(v_last) - this.computeDistanceFromVelocity(v_last);

    double v_current =
        m_direction
            ? -m_constraints.m_exponential.MaxAchievableVelocity(0) + 1e-9
            : m_constraints.m_exponential.MaxAchievableVelocity(0) - 1e-9;
    var x_current =
        second.computeDistanceFromVelocity(v_current) - this.computeDistanceFromVelocity(v_current);

    System.out.printf("%s, %s, %s, %s%n", x_last, v_last, x_current, v_current);

    while (Math.abs(v_current - v_last) > 1e-9 || Math.abs(x_current) > 1e-9) {
      var v_next = (v_last * x_current - v_current * x_last) / (x_current - x_last);
      var x_next =
          second.computeDistanceFromVelocity(v_next) - this.computeDistanceFromVelocity(v_next);
      v_last = v_current;
      x_last = x_current;
      v_current = v_next;
      x_current = x_next;
      System.out.printf("%s, %s, %s, %s%n", x_last, v_last, x_current, v_current);
    }

    return v_current;
  }
}
