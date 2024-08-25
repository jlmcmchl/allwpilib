// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

/**
 * A exponential curve-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on state-space model dynamics. To compute the reference
 * obeying this constraint, do the following.
 *
 * <p>Initialization:
 *
 * <pre><code>
 * ExponentialProfile.Constraints constraints =
 *   ExponentialProfile.Constraints.fromCharacteristics(kMaxV, kV, kA);
 * MotionProfile.State previousProfiledReference =
 *   new MotionProfile.State(initialReference, 0.0);
 * ExponentialProfile profile = constraints.asMotionProfile();
 * </code></pre>
 *
 * <p>Run on update:
 *
 * <pre><code>
 * previousProfiledReference =
 * profile.calculate(timeSincePreviousUpdate, previousProfiledReference, unprofiledReference);
 * </code></pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `timeRemaining()`.
 */
public class ExponentialProfile extends MotionProfile<ExponentialCurve> {
  public ExponentialProfile(ExponentialCurve.Constraints constraints) {
    super(constraints, constraints);
  }

  public ExponentialProfile(
      ExponentialCurve.Constraints acceleratingConstraints,
      ExponentialCurve.Constraints brakingConstraints) {
    super(acceleratingConstraints, brakingConstraints);
  }
}
