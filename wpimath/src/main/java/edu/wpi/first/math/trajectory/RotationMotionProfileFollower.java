// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.MotionProfile.State;

/**
 * A specialized motion profile follower for rotational motion.
 *
 * <p>This class extends the functionality of a standard motion profile follower
 * to handle the unique characteristics of rotational motion, including proper handling
 * of angle wrapping and shortest-path determination between two angles.
 *
 */
public class RotationMotionProfileFollower {
  private final MotionProfile profile;
  private final State currentState = new State();
  private final State goalState = new State();

  /**
   * Creates a new RotationMotionProfileFollower for the specified profile.
   *
   * @param profile The motion profile to follow.
   */
  public RotationMotionProfileFollower(MotionProfile profile) {
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
   * <p>The position is specified as a Rotation2d to properly handle angles, and the
   * internal state is normalized to find the shortest path to the goal.
   *
   * @param position The current angular position.
   * @param velocity The current angular velocity in radians per second.
   */
  public void setCurrentState(Rotation2d position, double velocity) {
    this.currentState.position = position.getRadians();
    this.currentState.velocity = velocity;

    fixCurrent();
  }

  /**
   * Sets the goal state of the follower.
   *
   * <p>The position is specified as a Rotation2d to properly handle angles, and the
   * internal state is normalized to find the shortest path to the goal.
   *
   * @param position The goal angular position.
   * @param velocity The goal angular velocity in radians per second.
   */
  public void setGoalState(Rotation2d position, double velocity) {
    this.goalState.position = position.getRadians();
    this.goalState.velocity = velocity;

    fixCurrent();
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
   * Adjusts the current position to ensure the shortest path is taken to the goal.
   *
   * <p>This method handles angle wrapping by adding or subtracting 2π radians from
   * the current position to ensure the shortest angular path is followed.
   *
   * @return True if the current position was adjusted, false otherwise.
   */
  private boolean fixCurrent() {
    double currentTime = timeRemaining();

    double adjustment = 0;
    if (goalState.position >= currentState.position) {
      adjustment = 2 * Math.PI;
    } else {
      adjustment -= 2 * Math.PI;
    }
    currentState.position += adjustment;

    if (timeRemaining() < currentTime) {
      return true;
    }

    currentState.position -= adjustment;
    return false;
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
