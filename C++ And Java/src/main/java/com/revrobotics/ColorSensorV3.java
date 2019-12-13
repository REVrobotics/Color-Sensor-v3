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

package com.revrobotics;

import edu.wpi.first.wpilibj.I2C;

import java.util.List;
import java.util.ArrayList;

/**
 * REV Robotics Color Sensor V3
 */
public class ColorSensorV3 extends ColorSensorV3LowLevel {

    public static class ColorValues {
        public int red;
        public int green;
        public int blue;
        public int ir;
    }

    enum Color {
        red,
        green,
        blue,
        yellow,
        unknown
    };

    /**
     * Constructs a ColorSensor.
     *
     * @param port  The I2C port the color sensor is attached to
     */
    public ColorSensorV3(I2C.Port port) {
        super(port);

        possibleColors.add(blueCoeff);
        possibleColors.add(greenCoeff);
        possibleColors.add(redCoeff);
        possibleColors.add(yellowCoeff);

        if(!checkDeviceID()) {
            return;
        }

        initializeDevice();
    }

    /**
     * Get the most likely color. Works best when within 2 inches and 
     * perpendicular to surface of interest.
     * 
     * @return  Color enum of the most likely color, including unknown if
     *          the minimum threshold is not met
     */
    public Color GetColor() {
        NormColorValues cv = GetNormColorValues();
        Color mostLikelyColor = Color.unknown;
        double maxR = 0;

        for(CalibCoeff c : possibleColors) {
            double r = c.getConfidence(cv);
            if(r > m_confidenceLevel && r > maxR) {
                mostLikelyColor = c.getColor();
                maxR = r;
            }
        }

        return mostLikelyColor;
    }

    /**
     * Get the raw proximity value from the sensor ADC (11 bit). This value 
     * is largest when an object is close to the sensor and smallest when 
     * far away.
     * 
     * @return  Proximity measurement value, ranging from 0 to 2047
     */
    public int getProximity() {
        return read11BitRegister(Register.kProximityData);
    }

    /**
     * Get the raw color values from their respective ADCs (20-bit).
     * 
     * @return  ColorValues struct containing red, green, blue and IR values
     */
    public ColorValues getColorValues() {
        ColorValues colors = new ColorValues();

        colors.ir = getIR();
        colors.green = getGreen();
        colors.blue = getBlue();
        colors.red = getRed();

        return colors;
    }

    /**
     * Get the raw color value from the red ADC
     * 
     * @return  Red ADC value
     */
    public int getRed() {
        return read20BitRegister(Register.kDataRed);
    }

    /**
     * Get the raw color value from the green ADC
     * 
     * @return  Green ADC value
     */
    public int getGreen() {
        return read20BitRegister(Register.kDataGreen);
    }

    /**
     * Get the raw color value from the blue ADC
     * 
     * @return  Blue ADC value
     */
    public int getBlue() {
        return read20BitRegister(Register.kDataBlue);
    }

    /**
     * Get the raw color value from the IR ADC
     * 
     * @return  IR ADC value
     */
    public int getIR() {
        return read20BitRegister(Register.kDataInfrared);
    }

    public void setConfidence(double confidence) {
        m_confidenceLevel = confidence;
    }

    /**
     * Helper function to turn Color enums into user readable strings
     * 
     * @return  Human readable string describing the color
     */
    public static String ColorToString(Color c) {
        switch(c) {
            case red: 
                return "Red";
            case blue:
                return "blue";
            case green:
                return "green";
            case yellow:
                return "yellow";
            default:
                return "Unknown";
        }
    }

    private class NormColorValues {
        private double m_red = 0;
        private double m_green = 0;
        private double m_blue = 0;
        private double m_ir = 0;

        public NormColorValues(double blue, double green, double red, double ir) {
            m_red = red;
            m_green = green;
            m_blue = blue;
            m_ir = ir;
        }

        public NormColorValues(ColorValues cv) {
            int magn = cv.blue + cv.green + cv.red + cv.ir;
            if(magn > 0) {
                m_blue = (double)cv.blue / magn;
                m_green = (double)cv.green / magn;
                m_red = (double)cv.red / magn;
                m_ir = (double)cv.ir / magn;
            }
        }

        public double getRed() {
            return m_red;
        }

        public double getBlue() {
            return m_blue;
        }

        public double getGreen() {
            return m_green;
        }

        public double getIR() {
            return m_ir;
        }
    };

    private class CalibCoeff {
        private NormColorValues m_nc;
        private Color m_col;

        public CalibCoeff(NormColorValues colors, Color col) {
            m_nc = colors;
            m_col = col;
        }

        public Color getColor() {
            return m_col;
        }

        public double getConfidence(NormColorValues meas) {
            return 1 - Math.sqrt(Math.pow(meas.getRed() - m_nc.getRed(), 2) + 
                                 Math.pow(meas.getGreen() - m_nc.getGreen(), 2) + 
                                 Math.pow(meas.getBlue() - m_nc.getBlue(), 2) + 
                                 Math.pow(meas.getIR() - m_nc.getIR(), 2))/Math.sqrt(2);
        }
    }

    private NormColorValues GetNormColorValues() {
        return new NormColorValues(getColorValues());
    }

    private double m_confidenceLevel = 0.95;

    private final NormColorValues expectedBlueValue = new NormColorValues(0.435, 0.415, 0.133, 0.017);
    private final NormColorValues expectedGreenValue = new NormColorValues(0.241, 0.548, 0.189, 0.022);
    private final NormColorValues expectedRedValue = new NormColorValues(0.117, 0.319, 0.540, 0.024);
    private final NormColorValues expectedYellowValue = new NormColorValues(0.112, 0.529, 0.349, 0.010);

    private final CalibCoeff blueCoeff = new CalibCoeff(expectedBlueValue, Color.blue);
    private final CalibCoeff greenCoeff = new CalibCoeff(expectedGreenValue, Color.green);
    private final CalibCoeff redCoeff = new CalibCoeff(expectedRedValue, Color.red);
    private final CalibCoeff yellowCoeff = new CalibCoeff(expectedYellowValue, Color.yellow);

    private List<CalibCoeff> possibleColors = new ArrayList<CalibCoeff>();
}