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

#include "rev/ColorSensor.h"
#include "frc/DriverStation.h"

using namespace rev;

ColorSensor::ColorSensor(frc::I2C::Port port) 
    : m_i2c(port, kAddress) {

    if(!CheckDeviceID())
        return;

    InitializeDevice();
}

bool ColorSensor::CheckDeviceID() {
    uint8_t partID = 0;
    if(m_i2c.Read(kPartIDRegister, 1, &partID)) {
        frc::DriverStation::ReportError("Could not find REV color sensor");
        return false;
    }

    if(partID != kPartID) {
        frc::DriverStation::ReportError("Unknown device found with same I2C addres as REV color sensor");
        return false;
    }

    return true;
}

void ColorSensor::InitializeDevice() {
    m_i2c.Write(kMainCtrlRegister, 
        kRGBMode | kLightSensorEnable | kProximitySensorEnable);

    m_i2c.Write(kProximitySensorRateRegister, kProxRes11bit | kProxRate100ms);
    m_i2c.Write(kProximitySensorPulsesRegister, 32);
}

uint32_t ColorSensor::GetProximity() {
    return Read11BitRegister(kProximityDataRegister);
}

ColorSensor::ColorValues ColorSensor::GetColorValues() {
    uint8_t raw[12];
    ColorValues color = {};

    if(!m_i2c.Read(kDataInfraredRegister, 12, raw)) {
        color.IR = To20Bit(&raw[0]);
        color.Green = To20Bit(&raw[3]);
        color.Blue = To20Bit(&raw[6]);
        color.Red = To20Bit(&raw[9]);
    }

    return color;
}

uint32_t ColorSensor::GetRed() {
    return Read20BitRegister(kDataRedRegister);
}

uint32_t ColorSensor::GetGreen() {
    return Read20BitRegister(kDataGreenRegister);
}

uint32_t ColorSensor::GetBlue() {
    return Read20BitRegister(kDataBlueRegister);
}

uint32_t ColorSensor::GetIR() {
    return Read20BitRegister(kDataInfraredRegister);
}

void 
ColorSensor::ConfigureProximitySensorLED(ColorSensor::LEDPulseFrequency freq,
                                         ColorSensor::LEDCurrent curr, 
                                         uint8_t pulses) {
    m_i2c.Write(kProximitySensorLEDRegister, freq | curr);
    m_i2c.Write(kProximitySensorPulsesRegister, pulses);
}

void 
ColorSensor::ConfigureProximitySensor(ColorSensor::ProximitySensorResolution res, 
                                      ColorSensor::ProximitySensorMeasurementRate rate) {
    m_i2c.Write(kProximitySensorRateRegister, res | rate);
}

void ColorSensor::ConfigureColorSensor(ColorSensor::ColorSensorResolution res, 
                                       ColorSensor::ColorSensorMeasurementRate rate, 
                                       ColorSensor::GainFactor gain) {
    m_i2c.Write(kLightSensorMeasurementRateRegister, res | rate);
    m_i2c.Write(kLightSensorGainRegister, gain);
}

uint16_t ColorSensor::Read11BitRegister(uint8_t reg) {
    uint8_t raw[2];

    m_i2c.Read(reg, 2, raw);

    return To11Bit(raw);
}

uint32_t ColorSensor::Read20BitRegister(uint8_t reg) {
    uint8_t raw[3];

    m_i2c.Read(reg, 3, raw);

    return To20Bit(raw);
}