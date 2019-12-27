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

import java.lang.Math;
import java.util.ArrayList;
import java.util.Optional;

public class ColorMatch {

    private static double CalculateDistance(Color color1, Color color2) {
        double redDiff = color1.red - color2.red;
        double greenDiff = color1.green - color2.green;
        double blueDiff = color1.blue - color2.blue;

        return Math.sqrt((redDiff*redDiff + greenDiff*greenDiff + blueDiff*blueDiff)/2);
    }
    
    private static final double kDefaultConfidence = 0.95;
    private double m_confidenceLevel;
    private ArrayList<Color> m_colorsToMatch;

    public ColorMatch() {
        m_confidenceLevel = kDefaultConfidence;
    }

    public void AddColorMatch(Color color) {
        m_colorsToMatch.add(color);
    }

    public void SetConfidenceThreshold(double confidence) {
        if (confidence < 0) { 
            confidence = 0;
        } else if (confidence > 1) {
            confidence = 1;
        }
        m_confidenceLevel = confidence;
    }

    public Optional<Color> MatchColor(Color colorToMatch) {
        return MatchColor(colorToMatch, Double.valueOf(0.0));
    }

    public Optional<Color> MatchColor(Color colorToMatch, Double confidence) {
        Color color = MatchClosestColor(colorToMatch, confidence);
        if ( confidence > m_confidenceLevel ) {
            return Optional.of(color);
        }
        confidence = 0.0;
        return Optional.empty();
    }

    public Color MatchClosestColor(Color color, Double confidence) {
        double magnitude = color.red + color.blue + color.green;

        if (magnitude > 0.0 && m_colorsToMatch.size() > 0) {
            Color normalized = new Color(color.red / magnitude, color.green / magnitude, color.blue / magnitude);
            double minDistance = 1.0;
            int idx = 0;

            for (int i=0; i < m_colorsToMatch.size(); i++) {
                double targetDistance = CalculateDistance(m_colorsToMatch.get(i), normalized);

                if (targetDistance < minDistance) {
                    minDistance = targetDistance;
                    idx = i;
                }
            }
            confidence = 1.0 - minDistance;
            return m_colorsToMatch.get(idx);
        } else {
            confidence = 0.0;
            //return frc::Color::kBlack;
            return new Color(0, 0, 0);
        }
    }

}