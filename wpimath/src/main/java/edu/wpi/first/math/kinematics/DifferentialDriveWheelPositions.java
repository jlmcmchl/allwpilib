// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/** Represents the wheel distances for a differential drive drivetrain. */
public class DifferentialDriveWheelPositions
    implements Difference<DifferentialDriveWheelPositions>,
        Copy<DifferentialDriveWheelPositions>,
        Interpolatable<DifferentialDriveWheelPositions> {
  /** Distance measured by the left side of the robot. */
  public double leftMeters;

  /** Distance measured by the right side of the robot. */
  public double rightMeters;

  /** Constructs a DifferentialDriveWheelSpeeds with zeros for left and right speeds. */
  public DifferentialDriveWheelPositions() {}

  /**
   * Constructs a DifferentialDriveWheelSpeeds.
   *
   * @param leftMeters The left distance.
   * @param rightMeters The right distance.
   */
  public DifferentialDriveWheelPositions(double leftMeters, double rightMeters) {
    this.leftMeters = leftMeters;
    this.rightMeters = rightMeters;
  }

  @Override
  public String toString() {
    return String.format(
        "DifferentialDriveWheelPositions(Left: %.2f m, Right: %.2f m)", leftMeters, rightMeters);
  }

  @Override
  public DifferentialDriveWheelPositions copy() {
    return new DifferentialDriveWheelPositions(leftMeters, rightMeters);
  }

  @Override
  public DifferentialDriveWheelPositions minus(DifferentialDriveWheelPositions other) {
    return new DifferentialDriveWheelPositions(
        leftMeters - other.leftMeters, rightMeters - other.rightMeters);
  }

  @Override
  public DifferentialDriveWheelPositions interpolate(
      DifferentialDriveWheelPositions endValue, double t) {
    return new DifferentialDriveWheelPositions(
        MathUtil.interpolate(leftMeters, endValue.leftMeters, t),
        MathUtil.interpolate(rightMeters, endValue.rightMeters, t));
  }
}
