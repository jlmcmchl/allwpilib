// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/PWMSparkMax.h"

#include <hal/FRCUsageReporting.h>

#include "frc/smartdashboard/SendableRegistry.h"

using namespace frc;

PWMSparkMax::PWMSparkMax(int channel) : PWMSpeedController(channel) {
  SetBounds(2.003, 1.55, 1.50, 1.46, 0.999);
  SetPeriodMultiplier(kPeriodMultiplier_1X);
  SetSpeed(0.0);
  SetZeroLatch();

  HAL_Report(HALUsageReporting::kResourceType_RevSparkMaxPWM, GetChannel() + 1);
  SendableRegistry::GetInstance().SetName(this, "PWMSparkMax", GetChannel());
}
