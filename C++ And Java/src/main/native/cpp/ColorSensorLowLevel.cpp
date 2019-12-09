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

#include "rev/ColorSensorV3LowLevel.h"
#include "frc/DriverStation.h"

#include <stdint.h>

using namespace rev;


bool ColorSensorV3LowLevel::CheckDeviceID() {
    uint8_t partID = 0;
    if(Read(Register::kPartID, 1, &partID)) {
        frc::DriverStation::ReportError("Could not find REV color sensor");
        return false;
    }

    if(partID != kExpectedPartID) {
        frc::DriverStation::ReportError("Unknown device found with same I2C addres as REV color sensor");
        return false;
    }

    return true;
}

void ColorSensorV3LowLevel::InitializeDevice() {
    Write(Register::kMainCtrl, 
          static_cast<uint8_t>(MainCtrlFields::kRGBMode) | 
          static_cast<uint8_t>(MainCtrlFields::kLightSensorEnable) | 
          static_cast<uint8_t>(MainCtrlFields::kProximitySensorEnable));

    Write(Register::kProximitySensorRate, 
          static_cast<uint8_t>(ProximityResolution::k11bit) | 
          static_cast<uint8_t>(ProximityMeasurementRate::k100ms));

    Write(Register::kProximitySensorPulses, 32);
}

void 
ColorSensorV3LowLevel::ConfigureProximitySensorLED(LEDPulseFrequency freq,
                                                   LEDCurrent curr, 
                                                   uint8_t pulses) {
    Write(Register::kProximitySensorLED,
          static_cast<uint8_t>(freq) | 
          static_cast<uint8_t>(curr));

    Write(Register::kProximitySensorPulses, pulses);
}

void 
ColorSensorV3LowLevel::ConfigureProximitySensor(ProximityResolution res, 
                                                ProximityMeasurementRate rate) {
    Write(Register::kProximitySensorRate, 
          static_cast<uint8_t>(res) | 
          static_cast<uint8_t>(rate));
}

void ColorSensorV3LowLevel::ConfigureColorSensor(ColorResolution res, 
                                                 ColorMeasurementRate rate) {
    Write(Register::kLightSensorMeasurementRate, 
          static_cast<uint8_t>(res) | 
          static_cast<uint8_t>(rate));
}

uint16_t ColorSensorV3LowLevel::Read11BitRegister(Register reg) {
    uint8_t raw[2];

    m_i2c.Read(static_cast<uint8_t>(reg), 2, raw);

    return To11Bit(raw);
}

uint32_t ColorSensorV3LowLevel::Read20BitRegister(Register reg) {
    uint8_t raw[3];

    m_i2c.Read(static_cast<uint8_t>(reg), 3, raw);

    return To20Bit(raw);
}