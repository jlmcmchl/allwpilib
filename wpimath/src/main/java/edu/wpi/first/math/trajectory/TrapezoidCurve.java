package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

public class TrapezoidCurve extends MotionCurve<TrapezoidCurve> {
  private final Constraints m_constraints;
  private final State m_initialState;
  private final double m_acceleration;

  public TrapezoidCurve(Constraints constraints, State state, double acceleration) {
    m_constraints = constraints;
    m_initialState = state;
    m_acceleration = acceleration;
  }

  public static class Constraints
      extends edu.wpi.first.math.trajectory.MotionCurve.Constraints<TrapezoidCurve> {
    public final double maxAcceleration;

    public Constraints(double acceleration) {
      this.maxAcceleration = acceleration;
    }

    public Constraints(double maxVelocity, double acceleration) {
      this.maxAcceleration = acceleration;

      withMaxVelocity(maxVelocity);
    }

    public TrapezoidCurve throughState(State state, boolean direction) {
      return new TrapezoidCurve(this, state, direction ? -maxAcceleration : maxAcceleration);
    }

    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }

    @Override
    public TrapezoidProfile asMotionProfile() {
      return new TrapezoidProfile(this);
    }
  }

  @Override
  public double computeDistanceFromVelocity(double velocity) {
    return m_initialState.position
        + (velocity * velocity - m_initialState.velocity * m_initialState.velocity)
            / (2 * m_acceleration);
  }

  @Override
  public double timeToState(State goal) {
    return (goal.velocity - m_initialState.velocity) / m_acceleration;
  }

  @Override
  public double computeVelocityFromTime(double t) {
    return m_initialState.velocity + m_acceleration * t;
  }

  @Override
  public double computeDistanceFromTime(double t) {
    return m_initialState.position + m_initialState.velocity * t + m_acceleration * t * t / 2;
  }

  @Override
  public double intersectionVelocity(TrapezoidCurve other) {
    double positionDifference = other.m_initialState.position - m_initialState.position;
    double velocitySum =
        m_initialState.velocity * m_initialState.velocity
            + other.m_initialState.velocity * other.m_initialState.velocity;

    double sqrtTerm = Math.sqrt(m_acceleration * positionDifference + velocitySum / 2);

    // Calculate the sign based on the acceleration direction
    double sign = Math.signum(m_acceleration);

    return sign * sqrtTerm;
  }
}
