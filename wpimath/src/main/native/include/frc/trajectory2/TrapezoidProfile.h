// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory2/FullStateMotionProfile.h"
#include "frc/trajectory2/VelocityMotionProfile.h"
#include "units/acceleration.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"
#include "wpimath/MathShared.h"

namespace frc {

/**
 * A trapezoid-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * Initialization:
 *
 * @code{.cpp}
 * TrapezoidProfile::Constraints constraints(kMaxV, kMaxA);
 * MotionProfile::State previousProfiledReference(initialReference, 0.0);
 * TrapezoidProfile profile = constraints.asMotionProfile();
 * @endcode
 *
 * Run on update:
 *
 * @code{.cpp}
 * previousProfiledReference =
 *     profile.calculate(timeSincePreviousUpdate, previousProfiledReference,
 * unprofiledReference);
 * @endcode
 *
 * where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `calculate()` returns the
 * unprofiled reference unchanged.
 *
 * Otherwise, a timer can be started to provide monotonic values for
 * `calculate()` and to determine when the profile has completed via
 * `timeRemaining()`.
 */
template <class Distance>
class TrapezoidProfile {
 public:
  using Distance_t = units::unit_t<Distance>;
  using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<Velocity>;
  using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<Acceleration>;

  class Constraints;  // Forward declaration

  /**
   * A curve segment of a trapezoidal profile.
   */
  class Curve : public MotionProfile<Distance>::Curve {
   public:
    /**
     * Creates a new trapezoidal curve.
     *
     * @param constraints The constraints for the curve.
     * @param state The initial state for the curve.
     * @param acceleration The acceleration to apply.
     */
    Curve(const Constraints& constraints,
          const typename MotionProfile<Distance>::State& state,
          Acceleration_t acceleration)
        : m_constraints(constraints),
          m_initialState(state),
          m_acceleration(acceleration) {}

    Distance_t ComputeDistanceFromVelocity(Velocity_t velocity) const override;
    units::second_t TimeToState(
        const typename MotionProfile<Distance>::State& goal) const override;
    Velocity_t ComputeVelocityFromTime(units::second_t t) const override;
    Distance_t ComputeDistanceFromTime(units::second_t t) const override;
    Velocity_t IntersectionVelocity(
        const typename MotionProfile<Distance>::Curve& other) const override;

   private:
    const Constraints& m_constraints;
    const typename MotionProfile<Distance>::State m_initialState;
    const Acceleration_t m_acceleration;
  };

  /**
   * Constraints for a trapezoidal motion profile.
   */
  class Constraints
      : public MotionProfile<Distance>::template Constraints<Curve> {
   public:
    /** The maximum acceleration for the profile. */
    Acceleration_t maxAcceleration{0};

    /**
     * Creates a new set of trapezoidal profile constraints with both velocity
     * and acceleration limits.
     *
     * @param maxVelocity The maximum velocity.
     * @param acceleration The maximum acceleration.
     */
    constexpr Constraints(Velocity_t maxVelocity, Acceleration_t acceleration)
        : maxAcceleration(acceleration) {
      this->maxVelocity = maxVelocity;
      if (!std::is_constant_evaluated()) {
        wpi::math::MathSharedStore::ReportUsage(
            wpi::math::MathUsageId::kTrajectory_TrapezoidProfile, 1);
      }
    }

    /**
     * Creates a curve passing through the given state in the specified
     * direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether to move in the negative (true) or positive
     * (false) direction.
     * @return A new curve passing through the given state.
     */
    Curve ThroughState(const typename MotionProfile<Distance>::State& state,
                       bool direction) const override {
      return Curve(*this, state,
                   direction ? -maxAcceleration : maxAcceleration);
    };
  };

  /**
   * Creates a FullStateMotionProfile with the same constraints for both forward
   * and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile<Distance, Curve, Curve> FullState(
      const Constraints& constraints) {
    return FullStateMotionProfile<Distance, Curve, Curve>(constraints,
                                                          constraints);
  }

  /**
   * Creates a FullStateMotionProfile with different constraints for forward and
   * reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile<Distance, Curve, Curve> FullState(
      const Constraints& forwardConstraints,
      const Constraints& reverseConstraints) {
    return FullStateMotionProfile<Distance, Curve, Curve>(forwardConstraints,
                                                          reverseConstraints);
  }

  /**
   * Creates a VelocityMotionProfile with the same constraints for both forward
   * and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile<Distance, Curve, Curve> VelocityOnly(
      const Constraints& constraints) {
    return VelocityMotionProfile<Distance, Curve, Curve>(constraints,
                                                         constraints);
  }

  /**
   * Creates a VelocityMotionProfile with different constraints for forward and
   * reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile<Distance, Curve, Curve> VelocityOnly(
      const Constraints& forwardConstraints,
      const Constraints& reverseConstraints) {
    return VelocityMotionProfile<Distance, Curve, Curve>(forwardConstraints,
                                                         reverseConstraints);
  }
};

}  // namespace frc
