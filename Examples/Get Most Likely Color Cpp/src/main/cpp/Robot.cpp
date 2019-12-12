/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>

#include "rev/ColorSensorV3.h"

/**
 * This is a simple example to show how the Rev Color Sensor V3 can be used to
 * detect the colors on the provided swatch.
 */
class Robot : public frc::TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};

 public:
  void RobotPeriodic() {
    /**
     * The method GetColor() returns an enum of the most likely swatch color 
     * detected by the device, including Color::unknown if the confidence 
     * threshold is not met for any of the colors.
     * 
     * This method works best when the LED on the sensor is on and the chip is
     * within a few inches of the color swatch.
     */
    rev::ColorSensorV3::Color detectedColor = m_colorSensor.GetColor();

    /**
     * The library includes a static ColorToString() method which converts
     * Color enums into strings for debug prints and other human readable
     * outputs.
     * 
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutString("Detected Color", 
      rev::ColorSensorV3::ColorToString(detectedColor));
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif