#pragma once

#include "frc/ErrorBase.h"
#include "frc/I2C.h"

namespace rev {
class ColorSensor {
    public:
        struct ColorValues {
            uint32_t Red;
            uint32_t Green;
            uint32_t Blue;
            uint32_t IR;
        };

        explicit ColorSensor(frc::I2C::Port port, int deviceAddress = kAddress);
        ~ColorSensor() = default;
        ColorSensor(ColorSensor&&) = default;
        ColorSensor& operator=(ColorSensor&&) = default;

        uint16_t GetProximity();
        ColorValues GetColorValues();
        uint32_t GetRed();
        uint32_t GetGreen();
        uint32_t GetBlue();

    private:
        uint32_t to20bit(uint8_t *val) {
            return (((uint32_t)val[2] << 16) | ((uint32_t)val[1] << 8) | 
                ((uint32_t)val[0])) & 0x0FFF;
        }

        bool CheckID();

        frc::I2C m_i2c;

        static constexpr int kAddress = 0x52;
        static constexpr int kPartID = 0xC2;

        enum Registers {
            kMainCtrlRegister = 0x00,
            PS_LED = 0x01,
            kProximitySensorPulsesRegister = 0x02,
            kProximitySensorRateRegister = 0x03,
            LS_MEAS_RATE = 0x04,
            LS_Gain = 0x05,
            kPartIDRegister = 0x06,
            MAIN_STATUS = 0x07,
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

        enum ProximitySensorLedFields {
            kLEDPulseCurrent2ma = 0x00,
            kLEDPulseCurrent5ma = 0x01,
            kLEDPulseCurrent10ma = 0x02,
            kLEDPulseCurrent25ma = 0x03,
            kLEDPulseCurrent50ma = 0x04,
            kLEDPulseCurrent75ma = 0x05,
            kLEDPulseCurrent100ma = 0x06,
            kLEDPulseCurrent125ma = 0x07
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