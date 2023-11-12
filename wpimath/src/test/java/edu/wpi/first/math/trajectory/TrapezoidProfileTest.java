// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TrapezoidProfileTest {
  private static final double kDt = 0.01;
  private static final TrapezoidCurve.Constraints m_constraints = new TrapezoidCurve.Constraints(0.75).withMaxVelocity(1.75);


  /**
   * Asserts "val1" is less than or equal to "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   */
  private static void assertLessThanOrEquals(double expected, double actual) {
    assertTrue(actual <= expected, actual + " is greater than " + expected);
  }

  /**
   * Asserts "val1" is within "eps" of "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   * @param eps Tolerance for whether values are near to each other.
   */
  private static void assertNear(double expected, double actual, double eps) {
    assertTrue(
        Math.abs(expected - actual) <= eps,
        "Difference between " + expected + " and " + actual + " is greater than " + eps);
  }

  /**
   * Asserts "val1" is less than or within "eps" of "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   * @param eps Tolerance for whether values are near to each other.
   */
  private static void assertLessThanOrNear(double expected, double actual, double eps) {
    if (actual <= expected) {
      assertLessThanOrEquals(expected, actual);
    } else {
      assertNear(expected, actual, eps);
    }
  }

  private static MotionProfile.State checkDynamics(
      TrapezoidProfile profile, TrapezoidCurve.Constraints constraints, MotionProfile.State current, MotionProfile.State goal) {
    var next = profile.calculate(kDt, current, goal);

    var velocity_delta = next.velocity - current.velocity;

    double accel = velocity_delta / kDt;

    // System.out.printf("%s, %s, %s%n", next.position, next.velocity, accel);
    assertAll(
        () -> assertLessThanOrNear(m_constraints.maxAcceleration, Math.abs(accel), 1e-9),
        () -> assertLessThanOrNear(m_constraints.maxVelocity, Math.abs(next.velocity), 1e-9));

    return next;
  }

  @Test
  void reachesGoal() {
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
    MotionProfile.State goal = new MotionProfile.State(3, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 450; ++i) {
      state = checkDynamics(profile, m_constraints, state, goal);
    }
    assertEquals(goal, state);
  }

  // Tests that decreasing the maximum velocity in the middle when it is already
  // moving faster than the new max is handled correctly
  @Test
  void posContinuousUnderVelChange() {
    var constraints = m_constraints;
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    MotionProfile.State goal = new MotionProfile.State(12, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 1600; ++i) {
      if (i == 400) {
        constraints = new TrapezoidCurve.Constraints(0.75).withMaxVelocity(0.75);
        profile = new TrapezoidProfile(constraints);
      }

      state = checkDynamics(profile, constraints, state, goal);
    }

    assertEquals(state, goal);
  }

  // There is some somewhat tricky code for dealing with going backwards
  @Test
  void backwards() {
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
    MotionProfile.State goal = new MotionProfile.State(-2, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 400; ++i) {
      state = checkDynamics(profile, m_constraints, state, goal);
    }
    assertEquals(goal, state);
  }

  @Test
  void switchGoalInMiddle() {
    TrapezoidCurve.Constraints constraints =
        new TrapezoidCurve.Constraints(0.75).withMaxVelocity(0.75);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    MotionProfile.State goal = new MotionProfile.State(-2, 0);
    MotionProfile.State state = new MotionProfile.State();

    // System.out.printf("%s, %s%n", state.position, state.velocity);

    for (int i = 0; i < 200; ++i) {
      state = checkDynamics(profile, constraints, state, goal);
      // System.out.printf("%s, %s%n", state.position, state.velocity);
    }
    assertNotEquals(state, goal);

    goal = new MotionProfile.State();
    for (int i = 0; i < 550; ++i) {
      state = checkDynamics(profile, constraints, state, goal);
      // System.out.printf("%s, %s%n", state.position, state.velocity);
    }
    assertEquals(state, goal);
  }

  // Checks to make sure that it hits top speed
  @Test
  void topSpeed() {
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints);
    MotionProfile.State goal = new MotionProfile.State(5, 0);
    MotionProfile.State state = new MotionProfile.State();

    double maxSpeed = state.velocity;

    for (int i = 0; i < 550; ++i) {
      state = checkDynamics(profile, m_constraints, state, goal);
      maxSpeed = Math.max(maxSpeed, state.velocity);
    }

    MotionProfile.State finalState = state;
    double finalMaxSpeed = maxSpeed;
    assertAll(
        () -> assertEquals(goal, finalState),
        () -> assertNear(m_constraints.maxVelocity, finalMaxSpeed, 1e-9));
  }

  @Test
  void timingToCurrent() {
    TrapezoidCurve.Constraints constraints = new
  TrapezoidCurve.Constraints(0.75).withMaxVelocity(0.75);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    MotionProfile.State goal = new MotionProfile.State(4, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 400; i++) {
      state = profile.calculate(kDt, state, goal);
      assertNear(0, profile.timeRemaining(state, state), 2e-2);
    }
  }

  @Test
  void timingToGoal() {
   TrapezoidCurve.Constraints constraints = new
  TrapezoidCurve.Constraints(0.75).withMaxVelocity(0.75);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    MotionProfile.State goal = new MotionProfile.State(2, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      state = profile.calculate(kDt, state, goal);
      if (!reachedGoal && state.equals(goal)) {
        // Expected value using for loop index is just an approximation since
        // the time left in the profile doesn't increase linearly at the
        // endpoints
        assertNear(predictedTimeLeft, i / 100.0, 0.25);
        reachedGoal = true;
      }
    }
  }

  @Test
  void timingToNegativeGoal() {
    TrapezoidCurve.Constraints constraints = new
  TrapezoidCurve.Constraints(0.75).withMaxVelocity(0.75);
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    MotionProfile.State goal = new MotionProfile.State(-2, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      state = profile.calculate(kDt, state, goal);
      if (!reachedGoal && state.equals(goal)) {
        // Expected value using for loop index is just an approximation since
        // the time left in the profile doesn't increase linearly at the
        // endpoints
        assertNear(predictedTimeLeft, i / 100.0, 0.25);
        reachedGoal = true;
      }
    }
  }
}
