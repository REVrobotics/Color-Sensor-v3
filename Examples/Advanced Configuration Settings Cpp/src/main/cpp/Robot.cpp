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
 * This is an example that shows the advanced configuration settings available
 * for the BROADCOM APDS-9151, the chip at the heart of the REV Color Sensor V3
 * 
 * Please refer to the device datasheet for more information about these 
 * settings. 
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
  void RobotInit() {
    /**
     * Note that the configuration methods below should never be called in a 
     * loop. Writing to any of the configuration registers causes the device to
     * reset its state machine and start a new measurement... Putting these in
     * RobotPeriodic() would cause you to never get new data!
     */

    /**
     * By default, the color sensor will read RGB and IR values with 18-bits of 
     * resolution every 100ms. Both the resolution and the measurement rate can
     * be configured to a higher or lower setting from the default.
     * 
     * Please note that while resolution can be increased, there is a trade off 
     * with the amount of time required for the measurement to complete. The 
     * measurement rate can be configured to a value less that this, but it 
     * will have no effect on the actual update rate.
     * 
     * The time required for each resolution is provided below:
     *  20 bit – 400 ms.
     *  19 bit – 200 ms.
     *  18 bit – 100 ms (default).
     *  17 bit – 50 ms.
     *  16 bit – 25 ms.
     *  13 bit – 3.125 ms.
     */
    const auto csResolution = rev::ColorSensorV3::ColorResolution::k20bit;
    const auto csRate = rev::ColorSensorV3::ColorMeasurementRate::k500ms;
    m_colorSensor.ConfigureColorSensor(csResolution, csRate);

    /**
     * The resolution and measurement rate of the proximity sensor can also be 
     * configured. This library by default configures the proximity resolution 
     * to the maximum value of 11 bits and leaves the measurement rate at the 
     * default of 100ms.
     * 
     * The datasheet notes that the measurement rate can be configured to a 
     * value faster than possible for the proximity ADC, but does not specify
     * what that value is given the resolution. It is left as an exercise to
     * the reader to determine whether their desired measurement rate is too
     * fast for the resolution they set.
     */
    const auto proxRes = rev::ColorSensorV3::ProximityResolution::k11bit;
    const auto proxRate = rev::ColorSensorV3::ProximityMeasurementRate::k100ms;
    ConfigureProximitySensor(proxRes, proxRate);

    /**
     * The pulse frequency, current level, and number of pulses can be 
     * configured for the proximity sensor LED. Increasing these parameters
     * will make the proximity sensor saturate at closer distances to objects, 
     * while decreasing them will have the opposite effect.
     */
    const auto ledPulseFreq = rev::ColorSensorV3::LEDPulseFrequency::kFreq60kHz;
    const auto ledCurr = rev::ColorSensorV3::LEDCurrent::kPulse100mA;
    const uint8_t pulse = 32;
    ConfigureProximitySensorLED(ledPulseFreq, ledCurr, pulse);
  }

  void RobotPeriodic() {
    rev::ColorSensorV3::ColorValues cv = m_colorSensor.GetColorValues();
    uint32_t proximity = m_colorSensor.GetProximity();

    frc::SmartDashboard::PutNumber("Red", cv.Red);
    frc::SmartDashboard::PutNumber("Green", cv.Green);
    frc::SmartDashboard::PutNumber("Blue", cv.Blue);
    frc::SmartDashboard::PutNumber("IR", cv.IR);
    frc::SmartDashboard::PutNumber("Proximity", proximity);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif