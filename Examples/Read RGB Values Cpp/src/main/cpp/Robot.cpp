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
 * This is a simple example to show the values that can be read from the REV
 * Color Sensor V3
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
     * The method GetColorValues() returns a ColorValues struct containing the
     * raw red, green, blue and IR values read from the chip. By default the 
     * sensor outputs these as 18-bit values and updates them every 100ms.
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     * 
     * The GetColorValues() method should generally be used in place of the 
     * individual getter methods for red green blue and IR as it uses the block 
     * read functionality of the chip to get all the values in a single I2C 
     * transaction which will be much faster than calling all of the methods 
     * individually.
     */
    rev::ColorSensorV3::ColorValues cv = m_colorSensor.GetColorValues();

    frc::SmartDashboard::PutNumber("Red", cv.Red);
    frc::SmartDashboard::PutNumber("Green", cv.Green);
    frc::SmartDashboard::PutNumber("Blue", cv.Blue);
    frc::SmartDashboard::PutNumber("IR", cv.IR);

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    uint32_t proximity = m_colorSensor.GetProximity();

    frc::SmartDashboard::PutNumber("Proximity", proximity);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif