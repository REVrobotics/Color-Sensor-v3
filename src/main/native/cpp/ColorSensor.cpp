#include "rev/ColorSensor.h"
#include <iostream>

using namespace rev;

ColorSensor::ColorSensor(frc::I2C::Port port, int deviceAddress) 
    : m_i2c(port, deviceAddress) {

    if(!CheckID())
        return;

    m_i2c.Write(kMainCtrlRegister, 
                kRGBMode | kLightSensorEnable | kProximitySensorEnable);

    m_i2c.Write(kProximitySensorPulsesRegister, 1);
    m_i2c.Write(kProximitySensorRateRegister, kProxRes11bit | kProxRate100ms);
    // TODO: register HAL
}

bool ColorSensor::CheckID() {
    uint8_t partID = 0;
    if(m_i2c.Read(kPartIDRegister, 1, &partID)) {
        std::cerr << "Nothing read for part id!\n";
        return false;
    }

    if(partID != kPartID) {
        std::cerr << "Part ID does not match expected!\n";
        return false;
    }

    return true;
}

uint16_t ColorSensor::GetProximity() {
    uint8_t prox[2];

    if(m_i2c.Read(kProximityDataRegister, 2, prox))
        return 0;

    return ((uint16_t)prox[1] << 8) | prox[0]; // TODO: overflow bit
}

ColorSensor::ColorValues ColorSensor::GetColorValues() {
    uint8_t raw[12];
    ColorValues color;

    if(!m_i2c.Read(kDataInfraredRegister, 12, raw)) {
        color.IR = to20bit(&raw[0]);
        color.Green = to20bit(&raw[3]);
        color.Blue = to20bit(&raw[6]);
        color.Red = to20bit(&raw[9]);
    }

    return color;
}

uint32_t ColorSensor::GetRed() {
    uint8_t raw[3];
    if(m_i2c.Read(kDataRedRegister, 3, raw)) {
        return 0;
    }

    return to20bit(raw);
}

uint32_t ColorSensor::GetGreen() {
    uint8_t raw[3];
    if(m_i2c.Read(kDataGreenRegister, 3, raw)) {
        return 0;
    }

    return to20bit(raw);
}

uint32_t ColorSensor::GetBlue() {
    uint8_t raw[3];

    if(m_i2c.Read(kDataBlueRegister, 3, raw)) {
        return 0;
    }

    return to20bit(raw);
}