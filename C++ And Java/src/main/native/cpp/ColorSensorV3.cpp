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

#include "rev/ColorSensorV3.h"
#include "frc/DriverStation.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>
#include <iomanip>

using namespace rev;

std::string rev::ColorToString(ColorSensorV3::Color cv) {
    static const std::vector<std::string> 
        ColorNames = {"Red", "Green", "Blue", "Yellow", "Unknown"};
    return ColorNames[static_cast<int>(cv)];
}

ColorSensorV3::ColorSensorV3(frc::I2C::Port port)
    : ColorSensorV3LowLevel(port), m_confidenceLevel(kDefaultConfidence) {

    if(!CheckDeviceID())
        return;

    InitializeDevice();
    SetGain(kDefaultGain);
}

uint32_t ColorSensorV3::GetProximity() {
    return Read11BitRegister(Register::kProximityData);
}

ColorSensorV3::ColorValues ColorSensorV3::GetColorValues() {
    uint8_t raw[12];
    ColorValues color = {};

    if(!Read(Register::kDataInfrared, 12, raw)) {
        color.IR = To20Bit(&raw[0]);
        color.Green = To20Bit(&raw[3]);
        color.Blue = To20Bit(&raw[6]);
        color.Red = To20Bit(&raw[9]);
    }

    return color;
}

uint32_t ColorSensorV3::GetRed() {
    return Read20BitRegister(Register::kDataRed);
}

uint32_t ColorSensorV3::GetGreen() {
    return Read20BitRegister(Register::kDataGreen);
}

uint32_t ColorSensorV3::GetBlue() {
    return Read20BitRegister(Register::kDataBlue);
}

uint32_t ColorSensorV3::GetIR() {
    return Read20BitRegister(Register::kDataInfrared);
}

void ColorSensorV3::SetGain(ColorSensorV3::GainFactor gain) {
    Write(Register::kLightSensorGain, static_cast<uint8_t>(gain));
}

ColorSensorV3::Color ColorSensorV3::GetColor() {
    NormalizedColorValues cv = GetNormalizedColorValues();
    // printf("B: %0.3f, G: %0.3f, R: %0.3f, I: %0.3f\n", cv.Blue, cv.Green, cv.Red, cv.IR);
    Color mostLikelyColor = Color::unknown;
    double maxR = 0;

    for(auto c : possibleColors) {
        double r = c.GetConfidence(cv);
        std::cout << ColorToString(c.GetColor()) << ": " << std::setprecision(3) << r << "\n";
        // TODO: ignore confidence if IR value is very large, indicating low light condition
        if(r > m_confidenceLevel && r > maxR) {
            mostLikelyColor = c.GetColor();
            maxR = r;
        }
    }

    return mostLikelyColor;
}

void ColorSensorV3::SetConfidence(double confidence) {
    if(confidence <= 1 && confidence >= 0)
        m_confidenceLevel = confidence;
}

ColorSensorV3::NormalizedColorValues::NormalizedColorValues(
    const ColorValues &cv) {
    uint32_t magn = cv.Red + cv.Blue + cv.Green + cv.IR;
    if(magn) {
        Red = (double)cv.Red / magn;
        Green = (double)cv.Green / magn;
        Blue = (double)cv.Blue / magn;
        IR = (double)cv.IR / magn;
    }
}

double ColorSensorV3::CalibCoeff::GetConfidence(const NormalizedColorValues &cv) const {
    return (sqrt(2) - sqrt(pow(cv.Red - m_nc.Red, 2) + 
                           pow(cv.Green - m_nc.Green, 2) + 
                           pow(cv.Blue - m_nc.Blue, 2) + 
                           pow(cv.IR - m_nc.IR, 2)))/sqrt(2);
}
