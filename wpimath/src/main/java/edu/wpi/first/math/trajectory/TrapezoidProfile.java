// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.trajectory;

public class TrapezoidProfile extends MotionProfile<TrapezoidCurve> {
  public TrapezoidProfile(TrapezoidCurve.Constraints constraints) {
    super(constraints, constraints);
  }

  public TrapezoidProfile(TrapezoidCurve.Constraints acceleratingConstraints, TrapezoidCurve.Constraints brakingConstraints) {
    super(acceleratingConstraints, brakingConstraints);
  }
}
