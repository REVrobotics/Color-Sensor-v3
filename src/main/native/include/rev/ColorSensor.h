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
            Gain1x = 0,
            Gain3x = 1,
            Gain6x = 2,
            Gain9x = 3,
            Gain18x = 4
        };

        /**
         * Constructs a ColorSensor.
         *
         * @param port  The I2C port the color sensor is attached to
         */
        explicit ColorSensor(frc::I2C::Port port);

        ColorSensor(ColorSensor&&) = default;
        ColorSensor& operator=(ColorSensor&&) = default;

        /**
         * Get the raw proximity value from the sensor ADC (11 bit). This value 
         * is largest when an object is close to the sensor and smallest when 
         * far away.
         * 
         * @return  Proximity measurement value, ranging from 0 to 2047
         */
        uint32_t GetProximity();

        /**
         * Get the raw color values from their respective ADCs (18-bit).
         * 
         * @return  ColorValues struct containing red, green, blue and IR values
         */
        ColorValues GetColorValues();

        /**
         * Get the raw color value from the red ADC (18-bit)
         * 
         * @return  Red ADC value
         */
        uint32_t GetRed();

        /**
         * Get the raw color value from the green ADC (18-bit)
         * 
         * @return  Green ADC value
         */
        uint32_t GetGreen();

        /**
         * Get the raw color value from the blue ADC (18-bit)
         * 
         * @return  Blue ADC value
         */
        uint32_t GetBlue();

        /**
         * Get the raw color value from the IR ADC (18-bit)
         * 
         * @return  IR ADC value
         */
        uint32_t GetIR();

        /**
         * Set the gain factor applied to the color ADC values. By default, the
         * gain factor is set to 3x when the chip boots up.
         * 
         * @param gain  Enum representing one of the possible gain values that
         * can be configured in the chip
         */
        void SetGain(GainFactor gain);

    private:
        uint32_t Read18BitRegister(uint8_t reg);
        uint32_t To18Bit(uint8_t *val) {
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
            kProximitySensorPulsesRegister = 0x02,
            kProximitySensorRateRegister = 0x03,
            kGainRegister = 0x05,
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

        enum ProximitySensorResolutionFields {
            kProxRes8bit = 0x00,
            kProxRes9bit = 0x08,
            kProxRes10bit = 0x10,
            kProxRes11bit = 0x18,
        };

        enum ProximitySensorMeasurementRateFields {
            kProxRate6ms = 1,
            kProxRate12ms = 2,
            kProxRate25ms = 3,
            kProxRate50ms = 4,
            kProxRate100ms = 5,
            kProxRate200ms = 6,
            kProxRate400ms = 7,
        };
};
}