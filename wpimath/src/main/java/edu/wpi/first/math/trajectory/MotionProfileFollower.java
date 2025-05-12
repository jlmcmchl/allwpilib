// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.trajectory.MotionProfile.State;

/**
 * A follower for motion profiles that tracks the progress of a profile over time.
 *
 * <p>This class maintains the current and goal states of a motion profile and provides methods
 * to update the profile state based on elapsed time. It's useful for implementing controllers
 * that need to follow a motion profile trajectory.
 *
 */
public class MotionProfileFollower {
  private final MotionProfile profile;
  private final State currentState = new State();
  private final State goalState = new State();

  /**
   * Creates a new MotionProfileFollower for the specified profile.
   *
   * @param profile The motion profile to follow.
   */
  public MotionProfileFollower(MotionProfile profile) {
    this.profile = profile;
  }

  /**
   * Gets the current state of the follower.
   *
   * @return A copy of the current position and velocity state.
   */
  public State getCurrentState() {
    return new State(currentState.position, currentState.velocity);
  }

  /**
   * Gets the goal state of the follower.
   *
   * @return A copy of the goal position and velocity state.
   */
  public State getGoalState() {
    return new State(goalState.position, goalState.velocity);
  }

  /**
   * Sets the current state of the follower.
   *
   * @param current The new current state.
   */
  public void setCurrentState(State current) {
    this.currentState.position = current.position;
    this.currentState.velocity = current.velocity;
  }

  /**
   * Sets the goal state of the follower.
   *
   * @param goal The new goal state.
   */
  public void setGoalState(State goal) {
    this.goalState.position = goal.position;
    this.goalState.velocity = goal.velocity;
  }

  /**
   * Updates the follower's state based on the elapsed time.
   *
   * <p>This method advances the motion profile by the specified time step and updates
   * the internal current state.
   *
   * @param dt The elapsed time since the last update, in seconds.
   * @return The new state after the update.
   */
  public State update(double dt) {
    var state = profile.calculate(dt, currentState, goalState);
    currentState.position = state.position;
    currentState.velocity = state.velocity;

    return state;
  }

  /**
   * Calculates the time remaining until the profile reaches the goal state.
   *
   * @return The time remaining in seconds.
   */
  public double timeRemaining() {
    return profile.timeRemaining(currentState, goalState);
  }
}
