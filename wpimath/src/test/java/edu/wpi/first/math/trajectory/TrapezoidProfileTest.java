// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class TrapezoidProfileTest {
  private static final double kDt = 0.01;
  private static final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(0.75).withMaxVelocity(1.75);

  private static void assertNear(MotionProfile.State val1, MotionProfile.State val2, double eps) {
    assertAll(
        () -> assertNear(val1.position, val2.position, eps),
        () -> assertNear(val1.position, val2.position, eps));
  }

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
      MotionProfile profile, MotionProfile.State current, MotionProfile.State goal) {
    var next = profile.calculate(kDt, current, goal);

    var velocity_delta = next.velocity - current.velocity;

    double accel = velocity_delta / kDt;

    double position_delta = next.position - current.position;
    double position_delta_expected = current.velocity * kDt + 0.5 * accel * kDt * kDt;

    assertAll(
        () -> assertLessThanOrNear(m_constraints.maxAcceleration, Math.abs(accel), 1e-9),
        () -> {
          if (Math.abs(current.velocity) <= m_constraints.maxVelocity) {
            assertLessThanOrNear(m_constraints.maxVelocity, Math.abs(next.velocity), 1e-9);
          }
        },
        () ->
            assertLessThanOrNear(
                Math.abs(position_delta_expected), Math.abs(position_delta), 2e-2));

    assertLessThanOrNear(m_constraints.maxAcceleration, Math.abs(accel), 1e-9);

    return next;
  }

  @Test
  void reachesGoal() {
    System.out.println("reachesGoal");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 2000; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // Tests that decreasing the maximum velocity in the middle when it is already
  // moving faster than the new max is handled correctly
  @Test
  void posContinuousUnderVelChange() {
    System.out.println("posContinuousUnderVelChange");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 2000; ++i) {
      if (i == 150) {
        profile =
            TrapezoidProfile.FullState(
                new TrapezoidProfile.Constraints(0.75).withMaxVelocity(0.75));
      }

      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // // Tests that decreasing the maximum velocity in the middle when it is already
  // // moving faster than the new max is handled correctly
  @Test
  void posContinuousUnderVelChangeBackward() {
    System.out.println("posContinuousUnderVelChangeBackward");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 2000; ++i) {
      if (i == 150) {
        profile =
            TrapezoidProfile.FullState(
                new TrapezoidProfile.Constraints(0.75).withMaxVelocity(0.75));
      }

      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // There is some somewhat tricky code for dealing with going backwards
  @Test
  void backwards() {
    System.out.println("backwards");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();

    for (int i = 0; i < 2000; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(goal, state);
  }

  @Test
  void switchGoalInMiddle() {
    System.out.println("switchGoalInMiddle");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-10, 0);
    MotionProfile.State state = new MotionProfile.State();
    for (int i = 0; i < 50; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertNotEquals(state, goal);

    goal = new MotionProfile.State(0.0, 0.0);
    for (int i = 0; i < 2000; ++i) {
      state = checkDynamics(profile, state, goal);
    }
    assertEquals(state, goal);
  }

  // Checks to make sure that it hits top speed
  @Test
  void topSpeed() {
    System.out.println("topSpeed");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(40, 0);
    MotionProfile.State state = new MotionProfile.State();
    double maxSpeed = 0;
    for (int i = 0; i < 3000; ++i) {
      state = checkDynamics(profile, state, goal);
      maxSpeed = Math.max(maxSpeed, state.velocity);
    }

    MotionProfile.State finalState = state;
    double finalMaxSpeed = maxSpeed;

    assertAll(
        () -> assertEquals(goal, finalState),
        () -> assertNear(m_constraints.maxVelocity, finalMaxSpeed, 1e-9));
  }

  @Test
  void topSpeedBackward() {
    System.out.println("topSpeedBackward");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();
    double maxSpeed = 0;
    for (int i = 0; i < 3000; ++i) {
      state = checkDynamics(profile, state, goal);
      maxSpeed = Math.min(maxSpeed, state.velocity);
    }

    MotionProfile.State finalState = state;
    double finalMaxSpeed = maxSpeed;

    assertAll(
        () -> assertEquals(goal, finalState),
        () -> assertNear(-m_constraints.maxVelocity, finalMaxSpeed, 1e-9));
  }

  @Test
  void largeInitialVelocity() {
    System.out.println("largeInitialVelocity");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(40, 0);
    MotionProfile.State state = new MotionProfile.State(0, 8);
    for (int i = 0; i < 2000; ++i) {
      state = checkDynamics(profile, state, goal);
    }

    assertEquals(state, goal);
  }

  @Test
  void largeNegativeInitialVelocity() {
    System.out.println("largeNegativeInitialVelocity");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State(0, -8);
    for (int i = 0; i < 2000; ++i) {
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
    System.out.println("testHeuristic");
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

    var profile = TrapezoidProfile.FullState(m_constraints);

    for (var testCase : testCases) {
      var direction = profile.shouldFlipInput(testCase.initial, testCase.goal);
      var state =
          m_constraints
              .throughState(testCase.initial, direction)
              .intersection(m_constraints.throughState(testCase.goal, !direction));
      // assertNear(testCase.inflectionPoint, state, 1e-3);
    }
  }

  @Test
  void timingToCurrent() {
    System.out.println("timingToCurrent");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(2, 0);
    MotionProfile.State state = new MotionProfile.State();
    for (int i = 0; i < 2000; i++) {
      state = checkDynamics(profile, state, goal);
      assertNear(profile.timeRemaining(state, state), 0, 2e-2);
    }
  }

  @Test
  void timingToGoal() {
    System.out.println("timingToGoal");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 2000; i++) {
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
    System.out.println("timingToNegativeGoal");
    MotionProfile profile = TrapezoidProfile.FullState(m_constraints);

    MotionProfile.State goal = new MotionProfile.State(-40, 0);
    MotionProfile.State state = new MotionProfile.State();

    double predictedTimeLeft = profile.timeRemaining(state, goal);
    boolean reachedGoal = false;
    for (int i = 0; i < 2000; i++) {
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
