package edu.wpi.first.math.kinematics;

public interface VelocityKinematics<VInput> {
  ChassisSpeeds toChassisSpeeds(VInput input);

  VInput toWheelSpeeds(ChassisSpeeds input);
}
