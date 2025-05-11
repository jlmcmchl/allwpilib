package edu.wpi.first.math.trajectory;

/**
 * A motion profile that controls both position and velocity states, with full state feedback.
 * 
 * <p>This class generates motion profiles that respect both position and velocity constraints,
 * creating smooth transitions between states. It can use different constraints for forward and
 * reverse motion, allowing for asymmetric acceleration and deceleration profiles.
 * 
 * <p>The profile automatically determines the optimal direction of travel and handles
 * cases where velocity limits are reached.
 */
public class FullStateMotionProfile extends MotionProfile {
  private final MotionProfile.Constraints m_forwardConstraints;
  private final MotionProfile.Constraints m_reverseConstraints;
  private final double m_maxVelocity;

  /**
   * Creates a new FullStateMotionProfile with the given constraints.
   *
   * @param forwardConstraints The constraints to use for forward motion.
   * @param reverseConstraints The constraints to use for reverse motion.
   */
  public FullStateMotionProfile(
      MotionProfile.Constraints forwardConstraints,
      MotionProfile.Constraints reverseConstraints) {
    m_forwardConstraints = forwardConstraints;
    m_reverseConstraints = reverseConstraints;
    m_maxVelocity = Math.min(m_forwardConstraints.maxVelocity, m_reverseConstraints.maxVelocity);
  }

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * <p>This method determines the optimal trajectory between the current state and goal state,
   * respecting all motion constraints, and returns the state at the specified time along
   * that trajectory.
   *
   * <p>The generated trajectory consists of up to three segments:
   * <ol>
   *   <li>An acceleration segment using the forward constraints</li>
   *   <li>An optional constant velocity segment at maximum velocity</li>
   *   <li>A deceleration segment using the reverse constraints</li>
   * </ol>
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  @Override
  public State calculate(double t, State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var forwardCurve = m_forwardConstraints.throughState(current, direction);
    var reverseCurve = m_reverseConstraints.throughState(goal, !direction);

    var intersection = forwardCurve.intersection(reverseCurve);

    var maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    if (Math.abs(current.velocity) > m_maxVelocity
        && Math.signum(current.velocity) == Math.signum(maxVelocity)) {
      forwardCurve = m_forwardConstraints.throughState(current, !direction);
    } else {
      forwardCurve = m_forwardConstraints.throughState(current, direction);
    }

    reverseCurve = m_reverseConstraints.throughState(intersection, !direction);

    if (Math.abs(intersection.velocity) < m_maxVelocity) {
      var timeAccelerating = forwardCurve.timeToState(intersection);

      if (timeAccelerating >= t) {
        return forwardCurve.stateAtTime(t);
      }

      var timeDecelerating = reverseCurve.timeToState(goal);

      if (timeAccelerating + timeDecelerating >= t) {
        return reverseCurve.stateAtTime(t - timeAccelerating);
      }

      return goal;
    }

    var forwardIntersectionWithMaxVelocity =
        new State(forwardCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    var timeAccelerating = forwardCurve.timeToState(forwardIntersectionWithMaxVelocity);

    if (timeAccelerating >= t) {
      return forwardCurve.stateAtTime(t);
    }

    var reverseIntersectionWithMaxVelocity =
        new State(reverseCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    reverseCurve =
        m_reverseConstraints.throughState(reverseIntersectionWithMaxVelocity, !direction);

    var timeAtMaxVelocity =
        (reverseIntersectionWithMaxVelocity.position - forwardIntersectionWithMaxVelocity.position)
            / maxVelocity;

    if (timeAccelerating + timeAtMaxVelocity >= t) {
      return new State(
          forwardIntersectionWithMaxVelocity.position + maxVelocity * (t - timeAccelerating),
          maxVelocity);
    }

    var timeDecelerating = reverseCurve.timeToState(goal);

    if (timeAccelerating + timeAtMaxVelocity + timeDecelerating >= t) {
      return reverseCurve.stateAtTime(t - (timeAccelerating + timeAtMaxVelocity));
    }

    return goal;
  }

  /**
   * Calculates the time remaining until the motion profile reaches the goal state.
   *
   * <p>This method determines the total time required to move from the current state to the
   * goal state along the optimal trajectory, respecting all motion constraints.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The time remaining until the goal state is reached.
   */
  @Override
  public double timeRemaining(State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var forwardCurve = m_forwardConstraints.throughState(current, direction);
    var reverseCurve = m_reverseConstraints.throughState(goal, !direction);

    var intersection = forwardCurve.intersection(reverseCurve);

    if (Math.abs(current.velocity) > m_maxVelocity) {
      forwardCurve = m_forwardConstraints.throughState(current, !direction);
    } else {
      forwardCurve = m_forwardConstraints.throughState(current, direction);
    }

    reverseCurve = m_reverseConstraints.throughState(intersection, !direction);

    if (Math.abs(intersection.velocity) < m_maxVelocity
        && Math.abs(current.velocity) < m_maxVelocity) {
      var timeAccelerating = forwardCurve.timeToState(intersection);

      var timeDecelerating = reverseCurve.timeToState(goal);

      return timeAccelerating + timeDecelerating;
    }

    var maxVelocity = direction ? -m_maxVelocity : m_maxVelocity;

    var forwardIntersectionWithMaxVelocity =
        new State(forwardCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    var timeAccelerating = forwardCurve.timeToState(forwardIntersectionWithMaxVelocity);

    var reverseIntersectionWithMaxVelocity =
        new State(reverseCurve.computeDistanceFromVelocity(maxVelocity), maxVelocity);

    reverseCurve =
        m_reverseConstraints.throughState(reverseIntersectionWithMaxVelocity, !direction);

    var timeAtMaxVelocity =
        (reverseIntersectionWithMaxVelocity.position - forwardIntersectionWithMaxVelocity.position)
            / maxVelocity;

    var timeDecelerating = reverseCurve.timeToState(goal);

    return timeAccelerating + timeAtMaxVelocity + timeDecelerating;
  }

  /**
   * Determines whether the input direction should be flipped based on the current and goal states.
   *
   * <p>This method examines the current position, velocity, and goal states to determine the
   * optimal direction for motion. It handles cases where direct paths may not be feasible
   * due to velocity constraints or when reversing direction is more efficient.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (reversed), false otherwise.
   */
  @Override
  public boolean shouldFlipInput(State current, State goal) {
    var xf = goal.position;
    var v0 = current.velocity;
    var vf = goal.velocity;

    var x_forward =
        m_forwardConstraints.throughState(current, false).computeDistanceFromVelocity(vf);
    var x_reverse =
        m_forwardConstraints.throughState(current, true).computeDistanceFromVelocity(vf);

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
