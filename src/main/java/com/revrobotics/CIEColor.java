/*
 * Copyright (c) 2020 REV Robotics
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

import java.lang.Math; 
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.ColorShim;

public class CIEColor {
    private double X;
    private double Y;
    private double Z;
    private double magnitude;

    //private final double IlluminantD65[] = {
    //    95.0489, 100.0, 108.8840
    //};

    private final double XYZtoRGB[]  = {
        3.2404542, -1.5371385, -0.4985314,
       -0.9692660,  1.8760108,  0.0415560,
        0.0556434, -0.2040259,  1.0572252
    };

    private static double CIERGB_f(double val) {
        return (val > 0.0031308) ? (1.055 * Math.pow(val, 1 / 2.4) - 0.055) : (12.92 * val);
    }

    private static double clamp(double x, double min, double max) {
        if (x > max) return max;
        if (x < min) return min;
        return x;
    }

    private Color ToRGB() {
        double _X = clamp(X / 100, 0.0, 1.0);
        double _Y = clamp(Y / 100, 0.0, 1.0);
        double _Z = clamp(Z / 100, 0.0, 1.0);
        double r = _X * XYZtoRGB[0] + _Y * XYZtoRGB[1] + _Z * XYZtoRGB[2];
        double g = _X * XYZtoRGB[3] + _Y * XYZtoRGB[4] + _Z * XYZtoRGB[5];
        double b = _X * XYZtoRGB[6] + _Y * XYZtoRGB[7] + _Z * XYZtoRGB[8];
    
        r = CIERGB_f(r);
        g = CIERGB_f(g);
        b = CIERGB_f(b);
    
        return new ColorShim(r,g,b);
    }

    /**
     * Get the X component of the color
     * 
     * @return  CIE X
     */
    public double getX() {
        return X;
    }

    /**
     * Get the Y component of the color
     * 
     * @return  CIE Y
     */
    public double getY() {
        return Y;
    }

    /**
     * Get the Z component of the color
     * 
     * @return  CIE Z
     */
    public double getZ() {
        return Z;
    }

    /**
     * Get the x calculated coordinate
     * of the CIE 19313 color space
     * 
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     * 
     * @return  CIE Yx
     */
    public double getYx() {
        return X / magnitude;
    }

    /**
     * Get the y calculated coordinate
     * of the CIE 19313 color space
     * 
     * https://en.wikipedia.org/wiki/CIE_1931_color_space
     * 
     * @return  CIE Yy
     */
    public double getYy() {
        return Y / magnitude;
    }

    public CIEColor(double x, double y, double z) {
        this.X = x;
        this.Y = y;
        this.Z = z;
        this.magnitude = x + y + z;
    }
}


