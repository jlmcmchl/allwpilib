// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.kinematics;

/** Represents the wheel speeds for a differential drive drivetrain. */
public class DifferentialDriveWheelDistances {
  /** Distance measured of the left side of the robot. */
  public double leftMeters;

  /** Distance measured of the right side of the robot. */
  public double rightMeters;

  /** Constructs a DifferentialDriveWheelSpeeds with zeros for left and right speeds. */
  public DifferentialDriveWheelDistances() {}

  /**
   * Constructs a DifferentialDriveWheelSpeeds.
   *
   * @param leftMeters The left distance.
   * @param rightMeters The right distance.
   */
  public DifferentialDriveWheelDistances(double leftMeters, double rightMeters) {
    this.leftMeters = leftMeters;
    this.rightMeters = rightMeters;
  }

  @Override
  public String toString() {
    return String.format(
        "DifferentialDriveWheelDistances(Left: %.2f m/s, Right: %.2f m/s)",
        leftMeters, rightMeters);
  }
}
