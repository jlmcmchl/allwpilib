package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

public class ExponentialCurve extends MotionCurve<ExponentialCurve> {
  private final State m_initialState;
  private final Constraints m_constraints;
  private final double m_input;

  public ExponentialCurve(Constraints constraints, State state, double input) {
    m_constraints = constraints;
    m_initialState = state;
    m_input = input;
  }

  public static class Constraints
      extends edu.wpi.first.math.trajectory.MotionCurve.Constraints<ExponentialCurve> {
    public final double maxInput;

    public final double A;
    public final double B;

    private Constraints(double maxInput, double A, double B) {
      this.maxInput = maxInput;
      this.A = A;
      this.B = B;
    }

    public static Constraints fromCharacteristics(double maxInput, double kV, double kA) {
      return new Constraints(maxInput, -kV / kA, 1.0 / kA);
    }

    public static Constraints fromStateSpace(double maxInput, double A, double B) {
      return new Constraints(maxInput, A, B);
    }

    public ExponentialCurve throughState(State state, boolean direction) {
      return new ExponentialCurve(this, state, direction ? -maxInput : maxInput);
    }

    public double MaxAchievableVelocity(double acceleration) {
      return (acceleration - B * maxInput) / A;
    }

    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }

    @Override
    public ExponentialProfile asMotionProfile() {
      return new ExponentialProfile(this);
    }
  }

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

  @Override
  public double computeVelocityFromTime(double t) {
    var A = m_constraints.A;
    var B = m_constraints.B;
    var u = m_input;

    return (m_initialState.velocity + B * u / A) * Math.exp(A * t) - B * u / A;
  }

  @Override
  public double computeDistanceFromTime(double t) {
    var A = m_constraints.A;
    var B = m_constraints.B;
    var u = m_input;

    return m_initialState.position
        + (-B * u * t + (m_initialState.velocity + B * u / A) * (Math.exp(A * t) - 1)) / A;
  }

  @Override
  public double intersectionVelocity(ExponentialCurve other) {
    var A = m_constraints.A;
    var B = m_constraints.B;
    var u = m_input;

    var U_dir = Math.signum(u);

    var position_delta = other.m_initialState.position - m_initialState.position;
    var velocity_delta = other.m_initialState.velocity - m_initialState.velocity;

    var scalar =
        (A * m_initialState.velocity + B * u) * (A * other.m_initialState.velocity - B * u);
    var power = -A / B / u * (A * position_delta - velocity_delta);

    var a = -A * A;
    var c = (B * B) * (u * u) + scalar * Math.exp(power);

    if (-1e-9 < c && c < 0) {
      // Numerical stability issue - the heuristic gets it right but c is around -1e-13
      return 0;
    }

    return U_dir * Math.sqrt(-c / a);
  }
}
