// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/trajectory/MotionProfile.h"
#include "frc/trajectory/FullStateMotionProfile.h"
#include "frc/trajectory/VelocityMotionProfile.h"

namespace frc {

/**
 * A exponential curve-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish, the intended usage
 * is to filter a reference's dynamics based on state-space model dynamics. To compute the reference
 * obeying this constraint, do the following .
 *
 * Initialization:
 *
 * @code{.cpp}
 * ExponentialProfile::Constraints constraints =
 *   ExponentialProfile::Constraints::FromCharacteristics(kMaxV, kV, kA);
 * MotionProfile::State previousProfiledReference(initialReference, 0.0);
 * ExponentialProfile profile = constraints.asMotionProfile();
 * @endcode
 *
 * Run on update:
 *
 * @code{.cpp}
 * previousProfiledReference =
 *     profile.calculate(timeSincePreviousUpdate, previousProfiledReference, unprofiledReference);
 * @endcode
 *
 * where `unprofiledReference` is free to change between calls. Note that when the unprofiled
 * reference is within the constraints, `calculate()` returns the unprofiled reference unchanged.
 *
 * Otherwise, a timer can be started to provide monotonic values for `calculate()` and to
 * determine when the profile has completed via `timeRemaining()`.
 */
template <class Distance>
class ExponentialProfile {
 public:

 /**
   * Constraints for an exponential profile.
   */
  class Constraints : public MotionProfile<Distance>::Constraints<MotionProfile<Distance>::Curve> {
   public:
    /** The maximum input value that can be applied to the system. */
    double maxInput;

    /** The state matrix coefficient for the system dynamics. */
    double A;
    
    /** The input matrix coefficient for the system dynamics. */
    double B;

    /**
     * Creates constraints from the physical system characteristics.
     *
     * @param maxInput The maximum input value.
     * @param kV The velocity gain (voltage per velocity).
     * @param kA The acceleration gain (voltage per acceleration).
     * @return A new Constraints object.
     */
    static Constraints FromCharacteristics(double maxInput, double kV, double kA) {
      return Constraints(maxInput, -kV / kA, 1.0 / kA);
    }

    /**
     * Creates constraints from state-space model parameters.
     *
     * @param maxInput The maximum input value.
     * @param A The state matrix coefficient for the system dynamics.
     * @param B The input matrix coefficient for the system dynamics.
     * @return A new Constraints object.
     */
    static Constraints FromStateSpace(double maxInput, double A, double B) {
      return Constraints(maxInput, A, B);
    }

    /**
     * Creates constraints from boundary conditions of the system.
     *
     * @param steadyStateVelocity The velocity at steady state with a unit input.
     * @param standstillAcceleration The acceleration at standstill with a unit input.
     * @return A new Constraints object.
     */
    static Constraints FromBoundaryConditions(double steadyStateVelocity,
                                            double standstillAcceleration) {
      return FromCharacteristics(1, 1 / steadyStateVelocity, 1 / standstillAcceleration);
    }

    /**
     * Creates a curve passing through the given state in the specified direction.
     *
     * @param state The state that the curve should pass through.
     * @param direction Whether to move in the positive (false) or negative (true) direction.
     * @return A new curve passing through the given state.
     */
    Curve ThroughState(const MotionProfile<Distance>::State& state, bool direction) const override;

    /**
     * Calculates the maximum achievable velocity for a given acceleration.
     *
     * @param acceleration The acceleration value to use.
     * @return The maximum achievable velocity.
     */
    double MaxAchievableVelocity(double acceleration) const {
      return (acceleration - B * maxInput) / A;
    }

    /**
     * Sets the maximum velocity constraint.
     *
     * @param velocity The maximum velocity.
     * @return This object for method chaining.
     */
    Constraints& WithMaxVelocity(double velocity) override {
      MotionProfile<Distance>::Constraints<MotionProfile<Distance>::Curve>::WithMaxVelocity(velocity);
      return *this;
    }

   private:
    /**
     * Creates a new set of exponential profile constraints.
     *
     * @param maxInput The maximum input value that can be applied.
     * @param A The state matrix coefficient for the system dynamics.
     * @param B The input matrix coefficient for the system dynamics.
     */
    Constraints(double maxInput, double A, double B)
        : maxInput(maxInput), A(A), B(B) {}
  };

  /**
   * A curve segment of an exponential profile.
   */
  class Curve : public MotionProfile<Distance>::Curve {
   public:
    /**
     * Creates a new exponential curve.
     *
     * @param constraints The constraints for the curve.
     * @param state The initial state for the curve.
     * @param input The input value to apply.
     */
    Curve(const Constraints& constraints, const MotionProfile<Distance>::State& state,
          double input);

    double ComputeDistanceFromVelocity(double velocity) const override;
    double TimeToState(const MotionProfile<Distance>::State& goal) const override;
    double ComputeVelocityFromTime(double t) const override;
    double ComputeDistanceFromTime(double t) const override;
    double IntersectionVelocity(const MotionProfile<Distance>::Curve& other) const override;

   private:
    const Constraints& m_constraints;
    const MotionProfile<Distance>::State m_initialState;
    const double m_input;
  };

  /**
   * Creates a FullStateMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile<Distance> FullState(const Constraints& constraints);

  /**
   * Creates a FullStateMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A FullStateMotionProfile with the specified constraints.
   */
  static FullStateMotionProfile<Distance> FullState(const Constraints& forwardConstraints,
                                        const Constraints& reverseConstraints);

  /**
   * Creates a VelocityMotionProfile with the same constraints for both forward and reverse motion.
   *
   * @param constraints The motion constraints to use for the profile.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile<Distance> VelocityOnly(const Constraints& constraints);

  /**
   * Creates a VelocityMotionProfile with different constraints for forward and reverse motion.
   *
   * @param forwardConstraints The motion constraints to use for forward motion.
   * @param reverseConstraints The motion constraints to use for reverse motion.
   * @return A VelocityMotionProfile with the specified constraints.
   */
  static VelocityMotionProfile<Distance> VelocityOnly(const Constraints& forwardConstraints,
                                          const Constraints& reverseConstraints);

};

}  // namespace frc 