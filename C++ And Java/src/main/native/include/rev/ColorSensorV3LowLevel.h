/*
 * Copyright (c) 2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "frc/I2C.h"

namespace rev {

class ColorSensorV3LowLevel {
public:
    static constexpr int kAddress = 0x52;
    static constexpr int kExpectedPartID = 0xC2;

    explicit ColorSensorV3LowLevel(frc::I2C::Port port) : m_i2c(port, kAddress) {};

    enum class LEDPulseFrequency {
        kFreq60kHz = 0x18,
        kFreq70kHz = 0x40,
        kFreq80kHz = 0x28,
        kFreq90kHz = 0x30,
        kFreq100kHz = 0x38,
    };

    enum class LEDCurrent {
        kPulse2mA = 0,
        kPulse5mA = 1,
        kPulse10mA = 2,
        kPulse25mA = 3,
        kPulse50mA = 4,
        kPulse75mA = 5,
        kPulse100mA = 6,
        kPulse125mA = 7,
    };

    enum class ProximityResolution {
        k8bit = 0x00,
        k9bit = 0x08,
        k10bit = 0x10,
        k11bit = 0x18,
    };

    enum class ProximityMeasurementRate {
        k6ms = 1,
        k12ms = 2,
        k25ms = 3,
        k50ms = 4,
        k100ms = 5,
        k200ms = 6,
        k400ms = 7,
    };

    enum class ColorResolution {
        k20bit = 0x00,
        k19bit = 0x08,
        k18bit = 0x10,
        k17bit = 0x18,
        k16bit = 0x20,
        k13bit = 0x28,
    };

    enum class ColorMeasurementRate {
        k25ms = 0,
        k50ms = 1,
        k100ms = 2,
        k200ms = 3,
        k500ms = 4,
        k1000ms = 5,
        k2000ms = 7,
    };

    /**
     * Configure the the IR LED used by the proximity sensor. 
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     * 
     * @param freq      The pulse modulation frequency for the proximity 
     *                  sensor LED
     * @param curr      The pulse current for the proximity sensor LED
     * @param pulses    The number of pulses per measurement of the 
     *                  proximity sensor LED
     */
    void ConfigureProximitySensorLED(LEDPulseFrequency freq, LEDCurrent curr, uint8_t pulses);
    
    /**
     * Configure the proximity sensor.
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * proximity sensor measurements.
     * 
     * @param res   Bit resolution output by the proximity sensor ADC.
     * @param rate  Measurement rate of the proximity sensor
     */
    void ConfigureProximitySensor(ProximityResolution res, ProximityMeasurementRate rate);
    
    /**
     * Configure the color sensor.
     * 
     * These settings are only needed for advanced users, the defaults 
     * will work fine for most teams. Consult the APDS-9151 for more 
     * information on these configuration settings and how they will affect
     * color sensor measurements.
     * 
     * @param res   Bit resolution output by the respective light sensor ADCs
     * @param rate  Measurement rate of the light sensor
     */
    void ConfigureColorSensor(ColorResolution res, ColorMeasurementRate rate);

    enum class Register {
        kMainCtrl = 0x00,
        kProximitySensorLED = 0x01,
        kProximitySensorPulses = 0x02,
        kProximitySensorRate = 0x03,
        kLightSensorMeasurementRate = 0x04,
        kLightSensorGain = 0x05,
        kPartID = 0x06,
        kProximityData = 0x08,
        kDataInfrared = 0x0A,
        kDataGreen = 0x0D,
        kDataBlue = 0x10,
        kDataRed = 0x13
    };

    enum class MainCtrlFields {
        kProximitySensorEnable = 0x01,
        kLightSensorEnable = 0x02,
        kRGBMode = 0x04
    };

    bool Write(Register reg, uint8_t data) {
        return m_i2c.Write(static_cast<uint8_t>(reg), data);
    }

    bool Read(Register reg, int count, uint8_t* data) {
        return m_i2c.Read(static_cast<uint8_t>(reg), count, data);
    }

    uint32_t To20Bit(uint8_t *val) {
        return (((uint32_t)val[2] << 16) | ((uint32_t)val[1] << 8) | ((uint32_t)val[0])) & 0x03FFFF;
    }
    
    uint16_t To11Bit(uint8_t *val) {
        return (((uint32_t)val[1] << 8) | val[0]) & 0x7FF;
    }

    uint32_t Read20BitRegister(Register reg);
    uint16_t Read11BitRegister(Register reg);

    bool CheckDeviceID();
    void InitializeDevice();

private:
    frc::I2C m_i2c;
};
}