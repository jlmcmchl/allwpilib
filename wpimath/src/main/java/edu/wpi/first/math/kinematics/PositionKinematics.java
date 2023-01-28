package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.geometry.Twist2d;

public interface PositionKinematics<DInput> {
  Twist2d toTwist2d(DInput input);
}
