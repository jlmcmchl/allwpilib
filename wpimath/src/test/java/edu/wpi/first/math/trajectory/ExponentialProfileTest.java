// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.List;
import org.junit.jupiter.api.Test;

class ExponentialProfileTest {
  private static final double kDt = 0.01;
  private static final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0, 2.5629, 0.43277);
  private static final ExponentialCurve.Constraints constraints =
      ExponentialCurve.Constraints.fromCharacteristics(12, 2.5629, 0.43277);

  /**
   * Asserts "val1" is within "eps" of "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   * @param eps Tolerance for whether values are near to each other.
   */
  private static void assertNear(double val1, double val2, double eps) {
    assertTrue(
        Math.abs(val1 - val2) <= eps,
        "Difference between " + val1 + " and " + val2 + " is greater than " + eps);
  }

  private static void assertNear(MotionProfile.State val1, MotionProfile.State val2, double eps) {
    assertAll(
        () -> assertNear(val1.position, val2.position, eps),
        () -> assertNear(val1.position, val2.position, eps));
  }

  private static MotionProfile.State checkDynamics(
      ExponentialProfile profile, MotionProfile.State current, MotionProfile.State goal) {
    var next = profile.calculate(kDt, current, goal);
    var signal = feedforward.calculateWithVelocities(current.velocity, next.velocity);

    assertTrue(Math.abs(signal) < constraints.maxInput + 1e-9);

    return next;
  }

  @Test
  void reachesGoal() {
    ExponentialProfile profile = constraints.asMotionProfile();

    MotionProfile.State goal = new MotionProfile.State(10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 450; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // Tests that decreasing the maximum velocity in the middle when it is already
  // moving faster than the new max is handled correctly
  @Test
  void posContinuousUnderVelChange() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 300; ++i) {
      if (i == 150) {
        profile =
            new ExponentialProfile(
                ExponentialCurve.Constraints.fromStateSpace(9, constraints.A, constraints.B));
      }

      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // // Tests that decreasing the maximum velocity in the middle when it is already
  // // moving faster than the new max is handled correctly
  @Test
  void posContinuousUnderVelChangeBackward() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 300; ++i) {
      if (i == 150) {
        profile =
            new ExponentialProfile(
                ExponentialCurve.Constraints.fromStateSpace(9, constraints.A, constraints.B));
      }

      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // There is some somewhat tricky code for dealing with going backwards
  @Test
  void backwards() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 400; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  @Test
  void switchGoalInMiddle() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();
    for (int i = 0; i < 50; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertNotEquals(state, goal);

    goal = new MotionProfile.State(0.0, 0.0);
    for (int i = 0; i < 100; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // Checks to make sure that it hits top speed
  @Test
  void topSpeed() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(40, 0);
    MotionProfile.State state = new MotionProfile.State();
    double maxSpeed = 0;
    for (int i = 0; i < 900; ++i) {
      state = checkDynamics(profile, state, goal);
      maxSpeed = Math.max(maxSpeed, state.velocity);
    }

    assertNear(constraints.MaxAchievableVelocity(0), maxSpeed, 10e-5);
    assertEquals(state, goal);
  }

  @Test
  void topSpeedBackward() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();
    double maxSpeed = 0;
    for (int i = 0; i < 900; ++i) {
      state = checkDynamics(profile, state, goal);
      maxSpeed = Math.min(maxSpeed, state.velocity);
    }

    assertNear(-constraints.MaxAchievableVelocity(0), maxSpeed, 10e-5);
    assertEquals(state, goal);
  }

  @Test
  void largeInitialVelocity() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(40, 0);
    MotionProfile.State state = new MotionProfile.State(0, 8);
    for (int i = 0; i < 900; ++i) {
      state = checkDynamics(profile, state, goal);
    }

    assertEquals(state, goal);
  }

  @Test
  void largeNegativeInitialVelocity() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State(0, -8);
    for (int i = 0; i < 900; ++i) {
      state = checkDynamics(profile, state, goal);
    }

    assertEquals(state, goal);
  }

  @SuppressWarnings("PMD.TestClassWithoutTestCases")
  static class TestCase {
    public final MotionProfile.State initial;
    public final MotionProfile.State goal;
    public final MotionProfile.State inflectionPoint;

    TestCase(
        MotionProfile.State initial,
        MotionProfile.State goal,
        MotionProfile.State inflectionPoint) {
      this.initial = initial;
      this.goal = goal;
      this.inflectionPoint = inflectionPoint;
    }
  }

  @Test
  void testHeuristic() {
    List<TestCase> testCases =
        List.of(
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(0.75, -4),
                new MotionProfile.State(1.3758, 4.4304)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(1.4103, 4),
                new MotionProfile.State(1.3758, 4.4304)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(0.75, -4),
                new MotionProfile.State(1.3758, 4.4304)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(1.4103, 4),
                new MotionProfile.State(1.3758, 4.4304)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(0.5, -2),
                new MotionProfile.State(0.4367, 3.7217)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(0.546, 2),
                new MotionProfile.State(0.4367, 3.7217)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(0.5, -2),
                new MotionProfile.State(0.5560, -2.9616)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(0.546, 2),
                new MotionProfile.State(0.5560, -2.9616)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(-0.75, -4),
                new MotionProfile.State(-0.7156, -4.4304)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(-0.0897, 4),
                new MotionProfile.State(-0.7156, -4.4304)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(-0.75, -4),
                new MotionProfile.State(-0.7156, -4.4304)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(-0.0897, 4),
                new MotionProfile.State(-0.7156, -4.4304)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(-0.5, -4.5),
                new MotionProfile.State(1.095, 4.314)),
            new TestCase(
                new MotionProfile.State(0.0, -4),
                new MotionProfile.State(1.0795, 4.5),
                new MotionProfile.State(-0.5122, -4.351)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(-0.5, -4.5),
                new MotionProfile.State(1.095, 4.314)),
            new TestCase(
                new MotionProfile.State(0.6603, 4),
                new MotionProfile.State(1.0795, 4.5),
                new MotionProfile.State(-0.5122, -4.351)),
            new TestCase(
                new MotionProfile.State(0.0, -8),
                new MotionProfile.State(0, 0),
                new MotionProfile.State(-0.1384, 3.342)),
            new TestCase(
                new MotionProfile.State(0.0, -8),
                new MotionProfile.State(-1, 0),
                new MotionProfile.State(-0.562, -6.792)),
            new TestCase(
                new MotionProfile.State(0.0, 8),
                new MotionProfile.State(1, 0),
                new MotionProfile.State(0.562, 6.792)),
            new TestCase(
                new MotionProfile.State(0.0, 8),
                new MotionProfile.State(-1, 0),
                new MotionProfile.State(-0.785, -4.346)));

    var profile = constraints.asMotionProfile();;

    for (var testCase : testCases) {
      var direction = profile.shouldFlipInput(testCase.initial, testCase.goal);
      var state =
          constraints
              .throughState(testCase.initial, direction)
              .intersection(constraints.throughState(testCase.goal, !direction));
      assertNear(testCase.inflectionPoint, state, 1e-3);
    }
  }

  @Test
  void timingToCurrent() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(2, 0);
    MotionProfile.State state = new MotionProfile.State();
    for (int i = 0; i < 400; i++) {
      state = checkDynamics(profile, state, goal);
      assertNear(profile.timeRemaining(state, state), 0, 2e-2);
    }
  }

  @Test
  void timingToGoal() {
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      state = checkDynamics(profile, state, goal);

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
    ExponentialProfile profile = constraints.asMotionProfile();;

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      state = checkDynamics(profile, state, goal);

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
