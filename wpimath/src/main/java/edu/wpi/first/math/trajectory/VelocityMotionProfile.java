package edu.wpi.first.math.trajectory;

/**
 * A motion profile that focuses only on velocity constraints without full position control.
 * 
 * <p>Unlike a full-state motion profile, this profile only attempts to reach the goal velocity
 * without ensuring the exact goal position is reached. It's useful for applications where
 * precise position control is less important than velocity tracking.
 */
public class VelocityMotionProfile extends MotionProfile {
  private final MotionProfile.Constraints m_forwardConstraints;
  private final MotionProfile.Constraints m_reverseConstraints;

  /**
   * Creates a new VelocityMotionProfile with the given constraints.
   *
   * @param forwardConstraints The constraints to use for forward motion (accelerating).
   * @param reverseConstraints The constraints to use for reverse motion (decelerating).
   */
  public VelocityMotionProfile(
      MotionProfile.Constraints forwardConstraints,
      MotionProfile.Constraints reverseConstraints) {
    m_forwardConstraints = forwardConstraints;
    m_reverseConstraints = reverseConstraints;
  }

  /**
   * Calculates the state of the motion profile at a specified time.
   *
   * <p>This method determines the optimal trajectory to reach the goal velocity,
   * respecting all motion constraints, and returns the state at the specified time
   * along that trajectory.
   *
   * <p>Once the goal velocity is reached, the position continues to increase at that velocity.
   *
   * @param t The time since the start of the profile.
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return The state at time t along the generated trajectory.
   */
  @Override
  public State calculate(double t, State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var constraints = direction ? m_forwardConstraints : m_reverseConstraints;
    var curve = constraints.throughState(current, direction);

    var timeToGoal = curve.timeToState(goal);

    if (timeToGoal <= t) {
      var distance = curve.computeDistanceFromVelocity(goal.velocity);
      distance += (t - timeToGoal) * goal.velocity;
      return new State(distance, goal.velocity);
    }

    return curve.stateAtTime(t);
  }

  /**
   * Calculates the time remaining until the motion profile reaches the goal velocity.
   *
   * <p>This method determines the total time required to transition from the current
   * velocity to the goal velocity, respecting all motion constraints.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (only velocity is considered).
   * @return The time remaining until the goal velocity is reached.
   */
  @Override
  public double timeRemaining(State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var constraints = direction ? m_forwardConstraints : m_reverseConstraints;
    var curve = constraints.throughState(current, direction);

    return curve.timeToState(goal);
  }

  /**
   * Determines whether the input direction should be flipped based on the current and goal velocities.
   *
   * <p>For a velocity profile, the direction only depends on whether we need to increase or
   * decrease velocity to reach the goal.
   *
   * @param current The current state (position and velocity).
   * @param goal The desired goal state (position and velocity).
   * @return True if the input direction should be flipped (when we need to decrease velocity),
   *         false otherwise.
   */
  @Override
  public boolean shouldFlipInput(State current, State goal) {
    return current.velocity > goal.velocity;
  }
}
