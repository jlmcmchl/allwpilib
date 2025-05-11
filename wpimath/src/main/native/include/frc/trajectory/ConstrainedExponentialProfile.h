// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory/MotionProfile.h"
#include "frc/trajectory/ExponentialProfile.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "frc/trajectory/FullStateMotionProfile.h"
#include "frc/trajectory/VelocityMotionProfile.h"

namespace frc {

/**
 * A motion profile that combines exponential and trapezoidal profiles.
 */
class ConstrainedExponentialProfile {
 public:
  /**
   * Creates a FullStateMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile FullState(const Constraints& constraints) {
    return FullState(constraints, constraints);
  }

  /**
   * Creates a FullStateMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile FullState(const Constraints& forwardConstraints,
                                        const Constraints& reverseConstraints) {
    return FullStateMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Creates a VelocityMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile VelocityOnly(const Constraints& constraints) {
    return VelocityOnly(constraints, constraints);
  }

  /**
   * Creates a VelocityMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile VelocityOnly(const Constraints& forwardConstraints,
                                          const Constraints& reverseConstraints) {
    return VelocityMotionProfile(forwardConstraints, reverseConstraints);
  }

  /**
   * Constraints for a constrained exponential profile.
   */
  class Constraints : public MotionProfile::Constraints<Curve> {
   public:
    /**
     * Creates a new set of constrained exponential profile constraints.
     *
     * @param trapezoid The trapezoidal profile constraints.
     * @param exponential The exponential profile constraints.
     */
    Constraints(const TrapezoidProfile::Constraints& trapezoid,
               const ExponentialProfile::Constraints& exponential)
        : m_trapezoid(trapezoid), m_exponential(exponential) {}

    /**
     * Creates a curve passing through the given state in the specified direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether to move in the negative (true) or positive (false) direction.
     * @return A new curve passing through the given state.
     */
    Curve ThroughState(const MotionProfile::State& state, bool direction) const override;

    /**
     * Gets the velocity at which the profile switches from trapezoidal to exponential.
     *
     * @return The switching velocity.
     */
    double SwitchingVelocity() const {
      return m_exponential.MaxAchievableVelocity(m_trapezoid.maxAcceleration);
    }

    /**
     * Sets the maximum velocity constraint.
     *
     * @param velocity The maximum velocity.
     * @return This object for method chaining.
     */
    Constraints& WithMaxVelocity(double velocity) override {
      MotionProfile::Constraints<Curve>::WithMaxVelocity(velocity);
      return *this;
    }

   private:
    const TrapezoidProfile::Constraints& m_trapezoid;
    const ExponentialProfile::Constraints& m_exponential;
  };

  /**
   * A curve segment of a constrained exponential profile.
   */
  class Curve : public MotionProfile::Curve {
   public:
    /**
     * Creates a new constrained exponential curve.
     *
     * @param constraints The constraints for the curve.
     * @param switchingState The state at which to switch between profiles.
     * @param initialState The initial state for the curve.
     * @param direction Whether to move in the negative (true) or positive (false) direction.
     */
    Curve(const Constraints& constraints, const MotionProfile::State& switchingState,
          const MotionProfile::State& initialState, bool direction);

    double ComputeDistanceFromVelocity(double velocity) const override;
    double TimeToState(const MotionProfile::State& goal) const override;
    double ComputeVelocityFromTime(double t) const override;
    double ComputeDistanceFromTime(double t) const override;
    double IntersectionVelocity(const MotionProfile::Curve& other) const override;

   private:
    const Constraints& m_constraints;
    const MotionProfile::State m_initialState;
    const MotionProfile::State m_switchingState;
    const TrapezoidProfile::Curve m_trapezoid;
    const ExponentialProfile::Curve m_exponential;
    const double m_timeToSwitchingState;
    const bool m_direction;
  };
};

}  // namespace frc 