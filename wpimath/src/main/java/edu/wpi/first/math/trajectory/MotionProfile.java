package edu.wpi.first.math.trajectory;

import java.util.Objects;

public class MotionProfile<T extends MotionCurve<T>> {
  private final MotionCurve.Constraints<T> m_firstCurveConstraints;
  private final MotionCurve.Constraints<T> m_secondCurveConstraints;
  private final double m_maxVelocity;

  public static class State {
    public double position;

    public double velocity;

    public State() {}

    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        return this.position == rhs.position && this.velocity == rhs.velocity;
      } else {
        return false;
      }
    }

    @Override
    public int hashCode() {
      return Objects.hash(position, velocity);
    }

    @Override
    public String toString() {
      return String.format("State(%s, %s)", position, velocity);
    }
  }

  public MotionProfile(
      MotionCurve.Constraints<T> firstCurveConstraints,
      MotionCurve.Constraints<T> secondCurveConstraints) {
    m_firstCurveConstraints = firstCurveConstraints;
    m_secondCurveConstraints = secondCurveConstraints;
    m_maxVelocity =
        Math.min(m_firstCurveConstraints.maxVelocity, m_secondCurveConstraints.maxVelocity);
  }

  public State calculate(double t, State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var firstCurve = m_firstCurveConstraints.throughState(current, direction);
    var secondCurve = m_secondCurveConstraints.throughState(goal, !direction);

    var intersection = firstCurve.intersection(secondCurve);

    var maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    if (Math.abs(current.velocity) > m_maxVelocity
        && Math.signum(current.velocity) == Math.signum(maxVelocity)) {
      firstCurve = m_firstCurveConstraints.throughState(current, !direction);
    } else {
      firstCurve = m_firstCurveConstraints.throughState(current, direction);
    }

    secondCurve = m_secondCurveConstraints.throughState(intersection, !direction);

    if (Math.abs(intersection.velocity) < m_maxVelocity) {
      var timeAccelerating = firstCurve.timeToState(intersection);

      if (timeAccelerating >= t) {
        return firstCurve.stateAtTime(t);
      }

      var timeDecelerating = secondCurve.timeToState(goal);

      if (timeAccelerating + timeDecelerating >= t) {
        return secondCurve.stateAtTime(t - timeAccelerating);
      }

      return goal;
    }

    var firstIntersectionWithMaxVelocity =
        new State(firstCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    var timeAccelerating = firstCurve.timeToState(firstIntersectionWithMaxVelocity);

    if (timeAccelerating >= t) {
      return firstCurve.stateAtTime(t);
    }

    var secondIntersectionWithMaxVelocity =
        new State(secondCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    secondCurve =
        m_secondCurveConstraints.throughState(secondIntersectionWithMaxVelocity, !direction);

    var timeAtMaxVelocity =
        (secondIntersectionWithMaxVelocity.position - firstIntersectionWithMaxVelocity.position)
            / maxVelocity;

    if (timeAccelerating + timeAtMaxVelocity >= t) {
      return new State(
          firstIntersectionWithMaxVelocity.position + maxVelocity * (t - timeAccelerating),
          maxVelocity);
    }

    var timeDecelerating = secondCurve.timeToState(goal);

    if (timeAccelerating + timeAtMaxVelocity + timeDecelerating >= t) {
      return secondCurve.stateAtTime(t - (timeAccelerating + timeAtMaxVelocity));
    }

    return goal;
  }

  public double timeRemaining(State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var firstCurve = m_firstCurveConstraints.throughState(current, direction);
    var secondCurve = m_secondCurveConstraints.throughState(goal, !direction);

    var intersection = firstCurve.intersection(secondCurve);

    if (Math.abs(current.velocity) > m_maxVelocity) {
      firstCurve = m_firstCurveConstraints.throughState(current, !direction);
    } else {
      firstCurve = m_firstCurveConstraints.throughState(current, direction);
    }

    secondCurve = m_secondCurveConstraints.throughState(intersection, !direction);

    if (Math.abs(intersection.velocity) < m_maxVelocity
        && Math.abs(current.velocity) < m_maxVelocity) {
      var timeAccelerating = firstCurve.timeToState(intersection);

      var timeDecelerating = secondCurve.timeToState(goal);

      return timeAccelerating + timeDecelerating;
    }

    var maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    var firstIntersectionWithMaxVelocity =
        new State(firstCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    var timeAccelerating = firstCurve.timeToState(firstIntersectionWithMaxVelocity);

    var secondIntersectionWithMaxVelocity =
        new State(secondCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    secondCurve =
        m_secondCurveConstraints.throughState(secondIntersectionWithMaxVelocity, !direction);

    var timeAtMaxVelocity =
        (secondIntersectionWithMaxVelocity.position - firstIntersectionWithMaxVelocity.position)
            / maxVelocity;

    var timeDecelerating = secondCurve.timeToState(goal);

    return timeAccelerating + timeAtMaxVelocity + timeDecelerating;
  }

  public boolean shouldFlipInput(State current, State goal) {
    var xf = goal.position;
    var v0 = current.velocity;
    var vf = goal.velocity;

    var x_forward =
        m_firstCurveConstraints.throughState(current, false).computeDistanceFromVelocity(vf);
    var x_reverse =
        m_firstCurveConstraints.throughState(current, true).computeDistanceFromVelocity(vf);

    if (Double.isNaN(x_forward)) {
      return xf < x_reverse;
    }

    if (Double.isNaN(x_reverse)) {
      return xf < x_forward;
    }

    var a = v0 >= 0;
    var b = vf >= 0;
    var c = xf >= x_forward;
    var d = xf >= x_reverse;

    return (a && !d) || (b && !c) || (!c && !d);
  }
}
