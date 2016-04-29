/**
 * FALKOLib - Fast Adaptive Laser Keypoint Orientation-invariant
 * Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
 *
 * This file is part of FALKOLib.
 *
 * FALKOLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * FALKOLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FALKOLib.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <falkolib/Common/HoughSpectrum.h>
#include <cassert>

using namespace std;

namespace falkolib {

    HoughSpectrum::HoughSpectrum()
    : thetaNum_(360),
    rhoNum_(1000),
    thetaStep_(0.0087266), rhoStep_(0.02),
    hough_(thetaNum_, rhoNum_),
    spectrum_(thetaNum_),
    orthoSpectrum_((thetaNum_ / 2) + (thetaNum_ % 2)),
    cosLut_(thetaNum_), sinLut_(thetaNum_) {
        for (unsigned int i = 0; i < thetaNum_; ++i) {
            cosLut_(i) = cos(thetaStep_ * i);
            sinLut_(i) = sin(thetaStep_ * i);
        }
    }

    HoughSpectrum::HoughSpectrum(double thetaStep, double rhoStep, double rhoMax)
    : thetaNum_(static_cast<unsigned int> (ceil(M_PI / thetaStep))),
    rhoNum_(2 * static_cast<unsigned int> (ceil(rhoMax / rhoStep))),
    thetaStep_(thetaStep), rhoStep_(rhoStep),
    hough_(thetaNum_, rhoNum_),
    spectrum_(thetaNum_),
    orthoSpectrum_((thetaNum_ / 2) + (thetaNum_ % 2)),
    cosLut_(thetaNum_), sinLut_(thetaNum_) {
        for (unsigned int i = 0; i < thetaNum_; ++i) {
            cosLut_(i) = cos(thetaStep_ * i);
            sinLut_(i) = sin(thetaStep_ * i);
        }
    }

    void HoughSpectrum::init(double thetaStep, double rhoStep, double rhoMax) {
        thetaStep_ = thetaStep;
        rhoStep_ = rhoStep;
        thetaNum_ = (unsigned int) ceil(M_PI / thetaStep);
        rhoNum_ = (unsigned int) ceil(rhoMax / rhoStep);

        hough_.resize(thetaNum_, rhoNum_);
        spectrum_.resize(thetaNum_);
        orthoSpectrum_.resize(thetaNum_ / 2);
        cosLut_.resize(thetaNum_);
        sinLut_.resize(thetaNum_);

        for (unsigned int i = 0; i < thetaNum_; ++i) {
            cosLut_(i) = cos(thetaStep_ * i);
            sinLut_(i) = sin(thetaStep_ * i);
        }
    }

} // end of namespace

