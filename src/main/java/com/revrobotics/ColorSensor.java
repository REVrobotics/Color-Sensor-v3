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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;;

/**
 * REV Robotics Color Sensor V3
 */
public class ColorSensor {
    private static final byte kAddress = 0x52;
    private static final byte kPartID = (byte) 0xC2;

    // register definitions
    private static final byte kMainCtrlRegister = 0x00;
    private static final byte kProximitySensorPulsesRegister = 0x02;
    private static final byte kProximitySensorRateRegister = 0x03;
    private static final byte kGainRegister = 0x05;
    private static final byte kPartIDRegister = 0x06;
    private static final byte kProximityDataRegister = 0x08;
    private static final byte kDataInfraredRegister = 0x0A;
    private static final byte kDataGreenRegister = 0x0D;
    private static final byte kDataBlueRegister = 0x10;
    private static final byte kDataRedRegister = 0x13;

    // register fields
    private static final byte kProximitySensorEnable = 0x01;
    private static final byte kLightSensorEnable = 0x02;
    private static final byte kRGBMode = 0x04;
    private static final byte kProxRes11bit = 0x18;
    private static final byte kProxRate100ms = 0x05;

    private I2C m_i2c;

    public enum GainFactor {
        Gain1x((byte) 0x00),
        Gain3x((byte) 0x01),
        Gain6x((byte) 0x02),
        Gain9x((byte) 0x03),
        Gain18x((byte) 0x04);

        /**
         * The integer value representing this enumeration.
         */
        public final byte value;

        GainFactor(byte value) {
            this.value = value;
        }
    }

    public static class ColorValues {
        public int red;
        public int green;
        public int blue;
        public int ir;
    }

    /**
     * Constructs a ColorSensor.
     *
     * @param port  The I2C port the color sensor is attached to
     */
    public ColorSensor(I2C.Port port) {
        m_i2c = new I2C(port, kAddress);

        if(!checkDeviceID()) {
            return;
        }

        initializeDevice();
    }

    /**
     * Get the raw proximity value from the sensor ADC (11 bit). This value 
     * is largest when an object is close to the sensor and smallest when 
     * far away.
     * 
     * @return  Proximity measurement value, ranging from 0 to 2047
     */
    public int getProximity() {
        return read11BitRegister(kProximityDataRegister);
    }

    /**
     * Get the raw color values from their respective ADCs (18-bit).
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
     * Get the raw color value from the red ADC (18-bit)
     * 
     * @return  Red ADC value
     */
    public int getRed() {
        return read18BitRegister(kDataRedRegister);
    }

    /**
     * Get the raw color value from the green ADC (18-bit)
     * 
     * @return  Green ADC value
     */
    public int getGreen() {
        return read18BitRegister(kDataGreenRegister);
    }

    /**
     * Get the raw color value from the blue ADC (18-bit)
     * 
     * @return  Blue ADC value
     */
    public int getBlue() {
        return read18BitRegister(kDataBlueRegister);
    }

    /**
     * Get the raw color value from the IR ADC (18-bit)
     * 
     * @return  IR ADC value
     */
    public int getIR() {
        return read18BitRegister(kDataInfraredRegister);
    }

    /**
     * Set the gain factor applied to the color ADC values. By default, the
     * gain factor is set to 3x when the chip boots up.
     * 
     * @param gain  Enum representing one of the possible gain values that
     * can be configured in the chip
     */
    public void setGain(GainFactor gain) {
        m_i2c.write(kGainRegister, gain.value);
    }

    public void close() {
        if (m_i2c != null) {
            m_i2c.close();
            m_i2c = null;
        }
    }

    private boolean checkDeviceID() {
        ByteBuffer raw = ByteBuffer.allocate(1);
        if(m_i2c.read(kPartIDRegister, 1, raw)) {
            DriverStation.reportError("Could not find REV color sensor", false);
            return false;
        }

        if(kPartID != raw.get()) {
            DriverStation.reportError("Unknown device found with same I2C addres as REV color sensor", false);
            return false;
        }

        return true;
    }

    private void initializeDevice() {
        m_i2c.write(kMainCtrlRegister, 
            kRGBMode | kLightSensorEnable | kProximitySensorEnable);

        m_i2c.write(kProximitySensorRateRegister, 
            kProxRes11bit | kProxRate100ms);

        m_i2c.write(kProximitySensorPulsesRegister, 32);
    }

    private int read11BitRegister(byte reg) {
        ByteBuffer raw = ByteBuffer.allocate(2);
    
        m_i2c.read(reg, 2, raw);
    
        raw.order(ByteOrder.LITTLE_ENDIAN);
        return raw.getShort() & 0x7FF;
    }

    private int read18BitRegister(byte reg) {
        ByteBuffer raw = ByteBuffer.allocate(4);
    
        m_i2c.read(reg, 3, raw);

        raw.order(ByteOrder.LITTLE_ENDIAN);
        return raw.getInt() & 0x03FFFF;
    }
}