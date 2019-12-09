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

#pragma once

#include "frc/ErrorBase.h"
#include "frc/I2C.h"

namespace rev {

/**
 * REV Robotics Color Sensor V3.
 *
 * This class allows access to a REV Robotics color sensor V3 on an I2C bus. 
 */
class ColorSensor : public frc::ErrorBase {
    public:
        struct ColorValues {
            uint32_t Red;
            uint32_t Green;
            uint32_t Blue;
            uint32_t IR;
        };

        enum GainFactor {
            kGain1x = 0,
            kGain3x = 1,
            kGain6x = 2,
            kGain9x = 3,
            kGain18x = 4
        };

        enum LEDPulseFrequency {
            kFreq60kHz = 0x18,
            kFreq70kHz = 0x40,
            kFreq80kHz = 0x28,
            kFreq90kHz = 0x30,
            kFreq100kHz = 0x38,
        };

        enum LEDCurrent {
            kPulse2mA = 0,
            kPulse5mA = 1,
            kPulse10mA = 2,
            kPulse25mA = 3,
            kPulse50mA = 4,
            kPulse75mA = 5,
            kPulse100mA = 6,
            kPulse125mA = 7,
        };

        enum ProximitySensorResolution {
            kProxRes8bit = 0x00,
            kProxRes9bit = 0x08,
            kProxRes10bit = 0x10,
            kProxRes11bit = 0x18,
        };

        enum ProximitySensorMeasurementRate {
            kProxRate6ms = 1,
            kProxRate12ms = 2,
            kProxRate25ms = 3,
            kProxRate50ms = 4,
            kProxRate100ms = 5,
            kProxRate200ms = 6,
            kProxRate400ms = 7,
        };

        enum ColorSensorResolution {
            kColorSensorRes20bit = 0x00,
            kColorSensorRes19bit = 0x08,
            kColorSensorRes18bit = 0x10,
            kColorSensorRes17bit = 0x18,
            kColorSensorRes16bit = 0x20,
            kColorSensorRes13bit = 0x28,
        };

        enum ColorSensorMeasurementRate {
            kColorRate25ms = 0,
            kColorRate50ms = 1,
            kColorRate100ms = 2,
            kColorRate200ms = 3,
            kColorRate500ms = 4,
            kColorRate1000ms = 5,
            kColorRate2000ms = 7,
        };

        /**
         * Constructs a ColorSensor.
         * 
         * Note that the REV Color Sensor is really two devices in one 
         * package - a color sensor providing red, green, blue and IR values,
         * and a proximity sensor.
         *
         * @param port  The I2C port the color sensor is attached to
         */
        explicit ColorSensor(frc::I2C::Port port);

        ColorSensor(ColorSensor&&) = default;
        ColorSensor& operator=(ColorSensor&&) = default;

        /**
         * Get the raw proximity value from the sensor ADC. This value 
         * is largest when an object is close to the sensor and smallest when 
         * far away.
         * 
         * @return  Proximity measurement value, ranging from 0 to 2047 in
         *          default configuration
         */
        uint32_t GetProximity();

        /**
         * Get the raw color values from their respective ADCs.
         * 
         * @return  ColorValues struct containing red, green, blue and IR values
         */
        ColorValues GetColorValues();

        /**
         * Get the raw color value from the red ADC
         * 
         * @return  Red ADC value
         */
        uint32_t GetRed();

        /**
         * Get the raw color value from the green ADC
         * 
         * @return  Green ADC value
         */
        uint32_t GetGreen();

        /**
         * Get the raw color value from the blue ADC
         * 
         * @return  Blue ADC value
         */
        uint32_t GetBlue();

        /**
         * Get the raw color value from the IR ADC
         * 
         * @return  IR ADC value
         */
        uint32_t GetIR();

        /**
         * Set the gain factor applied to color ADC measurements.
         * 
         * By default, the gain is set to 3x.
         * 
         * @param gain  Gain factor applied to color ADC measurements 
         *              measurements
         *
         */
        void SetGain(GainFactor gain);

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
        void ConfigureProximitySensor(ProximitySensorResolution res, ProximitySensorMeasurementRate rate);
        
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
        void ConfigureColorSensor(ColorSensorResolution res, ColorSensorMeasurementRate rate);


    private:
        uint32_t Read20BitRegister(uint8_t reg);
        uint32_t To20Bit(uint8_t *val) {
            return (((uint32_t)val[2] << 16) | ((uint32_t)val[1] << 8) | ((uint32_t)val[0])) & 0x03FFFF;
        }
        uint16_t Read11BitRegister(uint8_t reg);
        uint16_t To11Bit(uint8_t *val) {
            return (((uint32_t)val[1] << 8) | val[0]) & 0x7FF;
        }

        bool CheckDeviceID();
        void InitializeDevice();

        frc::I2C m_i2c;

        static constexpr int kAddress = 0x52;
        static constexpr int kPartID = 0xC2;

        enum Registers {
            kMainCtrlRegister = 0x00,
            kProximitySensorLEDRegister = 0x01,
            kProximitySensorPulsesRegister = 0x02,
            kProximitySensorRateRegister = 0x03,
            kLightSensorMeasurementRateRegister = 0x04,
            kLightSensorGainRegister = 0x05,
            kPartIDRegister = 0x06,
            kProximityDataRegister = 0x08,
            kDataInfraredRegister = 0x0A,
            kDataGreenRegister = 0x0D,
            kDataBlueRegister = 0x10,
            kDataRedRegister = 0x13
        };

        enum MainCtrlFields {
            kProximitySensorEnable = 0x01,
            kLightSensorEnable = 0x02,
            kRGBMode = 0x04
        };
};
}