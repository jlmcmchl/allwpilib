// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

public class ConstrainedExponentialProfile extends MotionProfile<ConstrainedExponentialCurve> {
  public ConstrainedExponentialProfile(ConstrainedExponentialCurve.Constraints constraints) {
    super(constraints, constraints);
  }

  public ConstrainedExponentialProfile(
      ConstrainedExponentialCurve.Constraints acceleratingConstraints,
      ConstrainedExponentialCurve.Constraints brakingConstraints) {
    super(acceleratingConstraints, brakingConstraints);
  }
}
