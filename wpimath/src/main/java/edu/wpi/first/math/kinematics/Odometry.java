package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Odometry<DInput extends Difference<DInput> & Copy<DInput>> {
  private final PositionKinematics<DInput> m_kinematics;

  private Pose2d m_poseMeters;
  private Rotation2d m_gyroOffset;

  private Rotation2d m_previousAngle;
  private DInput m_prevDistances;

  public Odometry(
      PositionKinematics<DInput> kinematics,
      Rotation2d gyroAngle,
      DInput currentMeasurement,
      Pose2d initialPoseMeters) {
    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();

    m_prevDistances = currentMeasurement.copy();

    MathSharedStore.reportUsage(MathUsageId.kOdometry_DifferentialDrive, 1);
  }

  public Odometry(
      PositionKinematics<DInput> kinematics, Rotation2d gyroAngle, DInput currentMeasurement) {
    this(kinematics, gyroAngle, currentMeasurement, new Pose2d());
  }

  public void resetPosition(Rotation2d gyroAngle, DInput distanceMeasurement, Pose2d poseMeters) {
    m_poseMeters = poseMeters;
    m_previousAngle = poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);

    m_prevDistances = distanceMeasurement.copy();
  }

  public Pose2d getPoseMeters() {
    return m_poseMeters;
  }

  public Pose2d update(Rotation2d gyroAngle, DInput distanceMeasurement) {
    DInput delta = distanceMeasurement.minus(m_prevDistances);

    m_prevDistances = distanceMeasurement.copy();

    var twist = m_kinematics.toTwist2d(delta);
    var angle = gyroAngle.plus(m_gyroOffset);

    twist.dtheta = angle.minus(m_previousAngle).getRadians();

    var newPose = m_poseMeters.exp(twist);

    m_previousAngle = angle;

    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
    return m_poseMeters;
  }
}
