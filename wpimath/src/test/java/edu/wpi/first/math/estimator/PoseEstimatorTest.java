// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimationTestUtils.SE2KinematicPrimitive;
import edu.wpi.first.math.estimator.PoseEstimationTestUtils.SE2Kinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.SamplingUtils;
import java.util.List;
import java.util.Random;
import java.util.TreeMap;
import org.junit.jupiter.api.Test;

class PoseEstimatorTest {
  private final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
  private final Vector<N3> stateNoiseStdDevs = stateStdDevs.div(50);
  private final Vector<N3> visionStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);

  private static final double visionUpdatePeriod = 0.2;
  private static final double visionUpdateDelay = 0.1;

  private static final double dt = 0.02;

  @Test
  void testAccuracy() {
    var kinematics = new SE2Kinematics(dt);

    var odometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());
    var oldOdometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());

    var oldEstimator = new OldPoseEstimator<>(kinematics, oldOdometry, stateStdDevs, visionStdDevs);
    var estimator = new PoseEstimator<>(odometry, stateStdDevs, visionStdDevs);

    var trajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
            new TrajectoryConfig(2, 2));

    testFollowTrajectory(
        kinematics,
        oldEstimator,
        estimator,
        trajectory,
        trajectory.getInitialPose(),
        trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters,
        dt,
        visionUpdatePeriod,
        visionUpdateDelay,
        true,
        stateNoiseStdDevs,
        visionStdDevs);
  }

  @Test
  void testBadInitialPose() {
    var kinematics = new SE2Kinematics(dt);

    var odometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());
    var oldOdometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());

    var estimator = new PoseEstimator<>(odometry, stateStdDevs, visionStdDevs);
    var oldEstimator = new OldPoseEstimator<>(kinematics, oldOdometry, stateStdDevs, visionStdDevs);

    var trajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
            new TrajectoryConfig(2, 2));

    for (int offset_direction_degs = 0; offset_direction_degs < 360; offset_direction_degs += 45) {
      for (int offset_heading_degs = 0; offset_heading_degs < 360; offset_heading_degs += 45) {
        var pose_offset = Rotation2d.fromDegrees(offset_direction_degs);
        var heading_offset = Rotation2d.fromDegrees(offset_heading_degs);

        var initial_pose =
            trajectory
                .getInitialPose()
                .plus(
                    new Transform2d(
                        new Translation2d(pose_offset.getCos(), pose_offset.getSin()),
                        heading_offset));

        testFollowTrajectory(
            kinematics,
            oldEstimator,
            estimator,
            trajectory,
            initial_pose,
            trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters,
            dt,
            visionUpdatePeriod,
            visionUpdateDelay,
            false,
            stateNoiseStdDevs,
            visionStdDevs);
      }
    }
  }

  void testFollowTrajectory(
      final SE2Kinematics kinematics,
      final OldPoseEstimator<SE2KinematicPrimitive> oldEstimator,
      final PoseEstimator<SE2KinematicPrimitive> estimator,
      final Trajectory trajectory,
      final Pose2d startingPose,
      final Pose2d endingPose,
      final double dt,
      final double visionUpdateRate,
      final double visionUpdateDelay,
      final boolean checkError,
      final Vector<N3> stateStdDevs,
      final Vector<N3> visionStdDevs) {
    oldEstimator.resetPosition(
        trajectory.getInitialPose().getRotation(),
        new SE2KinematicPrimitive(trajectory.getInitialPose()),
        startingPose);
    estimator.resetPosition(
        trajectory.getInitialPose().getRotation(),
        new SE2KinematicPrimitive(trajectory.getInitialPose()),
        startingPose);

    var rand = new Random(3538);

    double t = 0.0;

    final TreeMap<Double, Pose2d> visionUpdateQueue = new TreeMap<>();

    // System.out.printf(
    //     "%s, %s, %s, %s, %s, %s%n",
    //     oldEstimator.getEstimatedPosition().getX(),
    //     oldEstimator.getEstimatedPosition().getY(),
    //     oldEstimator.getEstimatedPosition().getRotation().getRadians(),
    //     estimator.getEstimatedPosition().getX(),
    //     estimator.getEstimatedPosition().getY(),
    //     estimator.getEstimatedPosition().getRotation().getRadians());

    double maxError = Double.NEGATIVE_INFINITY;
    double errorSum = 0;
    while (t <= trajectory.getTotalTimeSeconds()) {
      var groundTruthState = trajectory.sample(t);

      // We are due for a new vision measurement if it's been `visionUpdateRate` seconds since the
      // last vision measurement
      if (visionUpdateQueue.isEmpty() || visionUpdateQueue.lastKey() + visionUpdateRate < t) {
        var adjustment = SamplingUtils.sampleTwist2d(rand, visionStdDevs);
        Pose2d newVisionPose = groundTruthState.poseMeters.exp(adjustment);

        visionUpdateQueue.put(t, newVisionPose);
      }

      // We should apply the oldest vision measurement if it has been `visionUpdateDelay` seconds
      // since it was measured
      if (!visionUpdateQueue.isEmpty() && visionUpdateQueue.firstKey() + visionUpdateDelay < t) {
        var visionEntry = visionUpdateQueue.pollFirstEntry();

        System.out.println("applying vision update");

        oldEstimator.addVisionMeasurement(visionEntry.getValue(), visionEntry.getKey());
        estimator.addVisionMeasurement(visionEntry.getValue(), visionEntry.getKey());
      }

      var gyroAngle =
          groundTruthState
              .poseMeters
              .getRotation()
              .plus(SamplingUtils.sampleRotation2d(rand, stateStdDevs.get(2, 0)));

      var primitive =
          new SE2KinematicPrimitive(
              groundTruthState.poseMeters.exp(SamplingUtils.sampleTwist2d(rand, stateStdDevs)));

      oldEstimator.updateWithTime(t, gyroAngle, primitive);

      var xHat = estimator.updateWithTime(t, gyroAngle, primitive);

      double error =
          groundTruthState.poseMeters.getTranslation().getDistance(xHat.getTranslation());
      if (error > maxError) {
        maxError = error;
      }
      errorSum += error;

      // System.out.printf(
      //     "%s, %s, %s, %s, %s, %s%n",
      //     oldEstimator.getEstimatedPosition().getX(),
      //     oldEstimator.getEstimatedPosition().getY(),
      //     oldEstimator.getEstimatedPosition().getRotation().getRadians(),
      //     estimator.getEstimatedPosition().getX(),
      //     estimator.getEstimatedPosition().getY(),
      //     estimator.getEstimatedPosition().getRotation().getRadians());

      assertEquals(
          oldEstimator.getEstimatedPosition().getX(),
          estimator.getEstimatedPosition().getX(),
          1e-9);
      assertEquals(
          oldEstimator.getEstimatedPosition().getY(),
          estimator.getEstimatedPosition().getY(),
          1e-9);
      assertEquals(
          oldEstimator.getEstimatedPosition().getRotation().getSin(),
          estimator.getEstimatedPosition().getRotation().getSin(),
          1e-9);
      assertEquals(
          oldEstimator.getEstimatedPosition().getRotation().getCos(),
          estimator.getEstimatedPosition().getRotation().getCos(),
          1e-9);

      t += dt;
    }

    assertEquals(
        endingPose.getX(), estimator.getEstimatedPosition().getX(), 0.15, "Incorrect Final X");
    assertEquals(
        endingPose.getY(), estimator.getEstimatedPosition().getY(), 0.15, "Incorrect Final Y");
    assertEquals(
        endingPose.getRotation().getRadians(),
        estimator.getEstimatedPosition().getRotation().getRadians(),
        0.15,
        "Incorrect Final Theta");

    if (checkError) {
      assertEquals(
          0.0, errorSum / (trajectory.getTotalTimeSeconds() / dt), 0.15, "Incorrect mean error");
      assertEquals(0.0, maxError, 0.3, "Incorrect max error");
    }
  }

  @Test
  void testSimultaneousVisionMeasurements() {
    // This tests for multiple vision measurements appled at the same time. The expected behavior
    // is that all measurements affect the estimated pose. The alternative result is that only one
    // vision measurement affects the outcome. If that were the case, after 1000 measurements, the
    // estimated pose would converge to that measurement.
    var kinematics = new SE2Kinematics(dt);
    var odometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());
    var estimator = new PoseEstimator<>(odometry, stateStdDevs, visionStdDevs);

    estimator.updateWithTime(0, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()));

    var visionMeasurements =
        new Pose2d[] {
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          new Pose2d(3, 1, Rotation2d.fromDegrees(90)),
          new Pose2d(2, 4, Rotation2d.fromRadians(180)),
        };

    for (int i = 0; i < 1000; i++) {
      for (var measurement : visionMeasurements) {
        estimator.addVisionMeasurement(measurement, 0);
      }
    }

    for (var measurement : visionMeasurements) {
      var errorLog =
          "Estimator converged to one vision measurement: "
              + estimator.getEstimatedPosition().toString()
              + " -> "
              + measurement.toString();

      var dx = Math.abs(measurement.getX() - estimator.getEstimatedPosition().getX());
      var dy = Math.abs(measurement.getY() - estimator.getEstimatedPosition().getY());
      var dtheta =
          Math.abs(
              measurement.getRotation().getDegrees()
                  - estimator.getEstimatedPosition().getRotation().getDegrees());

      assertEquals(dx > 0.08 || dy > 0.08 || dtheta > 0.08, true, errorLog);
    }
  }

  @Test
  void testDiscardsStaleVisionMeasurements() {
    var kinematics = new SE2Kinematics(dt);
    var odometry =
        new Odometry<>(
            kinematics, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()), new Pose2d());
    var estimator = new PoseEstimator<>(odometry, stateStdDevs, visionStdDevs);

    double time = 0;

    // Add enough measurements to fill up the buffer
    for (; time < 4; time += 0.02) {
      estimator.updateWithTime(time, new Rotation2d(), new SE2KinematicPrimitive(new Pose2d()));
    }

    var odometryPose = estimator.getEstimatedPosition();

    // Apply a vision measurement made 3 seconds ago
    // This test passes if this does not cause a ConcurrentModificationException.
    estimator.addVisionMeasurement(
        new Pose2d(new Translation2d(10, 10), new Rotation2d(0.1)),
        1,
        VecBuilder.fill(0.1, 0.1, 0.1));

    assertEquals(odometryPose.getX(), estimator.getEstimatedPosition().getX(), "Incorrect Final X");
    assertEquals(odometryPose.getY(), estimator.getEstimatedPosition().getY(), "Incorrect Final Y");
    assertEquals(
        odometryPose.getRotation().getRadians(),
        estimator.getEstimatedPosition().getRotation().getRadians(),
        "Incorrect Final Theta");
  }
}
