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

#include "rev/ColorSensorV3LowLevel.h"
#include "frc/ErrorBase.h"
#include <vector>
#include <string>

namespace rev {

/**
 * REV Robotics Color Sensor V3.
 *
 * This class allows access to a REV Robotics color sensor V3 on an I2C bus. 
 */
class ColorSensorV3 : public ColorSensorV3LowLevel, public frc::ErrorBase {
public:
    struct ColorValues {
        uint32_t Red;
        uint32_t Green;
        uint32_t Blue;
        uint32_t IR;
    };

    enum class Color {
        red = 0,
        green = 1,
        blue = 2,
        yellow = 3,
        unknown = 4
    };

    /**
     * Constructs a ColorSensorV3.
     * 
     * Note that the REV Color Sensor is really two devices in one package:
     * a color sensor providing red, green, blue and IR values, and a proximity
     * sensor.
     *
     * @param port  The I2C port the color sensor is attached to
     */
    explicit ColorSensorV3(frc::I2C::Port port);

    ColorSensorV3(ColorSensorV3&&) = default;
    ColorSensorV3& operator=(ColorSensorV3&&) = default;

    /**
     * Get the most likely color. Works best when within 2 inches and 
     * perpendicular to surface of interest.
     * 
     * @return  Color enum of the most likely color, including unknown if
     *          the minimum threshold is not met
     */
    Color GetColor();

    /**
     * Set the confidence interval for determining color. Defaults to 0.95
     * 
     * @param confidence    A value between 0 and 1
     */
    void SetConfidence(double confidence);

    /**
     * Get the raw color values from their respective ADCs.
     * 
     * @return  ColorValues struct containing red, green, blue and IR values
     */
    ColorValues GetColorValues();

    /**
     * Get the raw proximity value from the sensor ADC. This value is largest 
     * when an object is close to the sensor and smallest when 
     * far away.
     * 
     * @return  Proximity measurement value, ranging from 0 to 2047 in
     *          default configuration
     */
    uint32_t GetProximity();

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
     */
    void SetGain(GainFactor gain);

    /**
     * Helper function to turn Color enums into user readable strings
     * 
     * @return  Human readable string describing the color
     */
    static std::string ColorToString(ColorSensorV3::Color cv);

private:
    struct NormColorValues {
        double Red;
        double Green;
        double Blue;
        double IR;

        NormColorValues(double blue, double green, double red, double ir) :
            Red(red), Green(green), Blue(blue), IR(ir) {}

        NormColorValues(const ColorValues &cv);
    };

    class CalibCoeff {
    public:
        CalibCoeff(NormColorValues colors, Color col) : 
            m_nc(colors), m_col(col) {}

        double GetConfidence(const NormColorValues &cv) const;

        Color GetColor() const { return m_col; }

    private:
        NormColorValues m_nc;
        Color m_col;
    };

    NormColorValues GetNormColorValues() { return NormColorValues(GetColorValues()); }

    const CalibCoeff blueCoeff{NormColorValues(0.435, 0.415, 0.133, 0.017), Color::blue};
    const CalibCoeff greenCoeff{NormColorValues(0.241, 0.548, 0.189, 0.022), Color::green};
    const CalibCoeff redCoeff{NormColorValues(0.117, 0.319, 0.540, 0.024), Color::red};
    const CalibCoeff yellowCoeff{NormColorValues(0.112, 0.529, 0.349, 0.010), Color::yellow};
    const std::vector<CalibCoeff> possibleColors = {blueCoeff, greenCoeff, redCoeff, yellowCoeff};

    double m_confidenceLevel;
    static constexpr double kDefaultConfidence = 0.95;
    static constexpr GainFactor kDefaultGain = GainFactor::k3x;
};

}