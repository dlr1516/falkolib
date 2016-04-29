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
#pragma once

#include <falkolib/Feature/KeypointExtractor.h>
#include <falkolib/Feature/OC.h>
#include <falkolib/Common/Point.h>
#include <falkolib/Common/GeomUtils.h>
#include <falkolib/Common/HoughSpectrum.h>

namespace falkolib {

    /**
     * @brief class representing a OC keypoint extractor engine 
     */
    class OCExtractor : public KeypointExtractor<OC> {
    public:

        enum CornerOrientation {
            NE = 0, NW, SW, SE
        };

        /**
         * @brief Constructor
         */
        OCExtractor();

        /**
         * @brief Extract OC keypoints from a given scan 	 
         * @param scan input laser scan
         * @param features OC keypoint extracted from given scan
         */
        void extract(const LaserScan& scan, std::vector<OC>& keypoints);

        /** @brief Set the tolerance distance to consider point aligned to corner edge. */
        inline void setTol(double _tol) {
            tol = _tol;
        };

        /** @brief Set the angular resolution. */
        inline void setAngleRes(double _angleRes) {
            angleRes = _angleRes;
            houghSpectrum.init(angleRes, rangeRes, rangeMax);
        };

        /** @brief Set the range/distance resolution used for Hough rho size. */
        inline void setRangeRes(double _rangeRes) {
            rangeRes = _rangeRes;
            houghSpectrum.init(angleRes, rangeRes, rangeMax);
        };

        /** @brief Set the maximum range for Hough rho cell grid. */
        inline void setRangeMax(double _rangeMax) {
            rangeMax = _rangeMax;
            houghSpectrum.init(angleRes, rangeRes, rangeMax);
        };

        /** @brief Set the non maxima suppression radius. */
        inline void setNMSRadius(double _nmsRadius) {
            nmsRadius = _nmsRadius;
        };

        /** @brief Set neighA parameter for neighborhood radius computation*/
        inline void setNeighA(double _neighA) {
            neighA = _neighA;
        };

        /** @brief Set neighB parameter for neighborhood radius computation*/
        inline void setNeighB(double _neighB) {
            neighB = _neighB;
        };

        /** @brief Set minimum neighborhood size for each corner side*/
        inline void setNeighMinPoint(int _neighMinPoint) {
            neighMinPoint = _neighMinPoint;
        };


    private:
        HoughSpectrum houghSpectrum;
        double tol;
        double angleRes;
        double rangeRes;
        double rangeMax;
        double nmsRadius;
        double neighA;
        double neighB;
        int neighMinPoint;

        /** @brief Computes the dominant direction of a scan using Hough Spectrum.
         */
        double computeDominantAngle(const std::vector<Point2d>& points);

        /** @brief Rotates points. 
         */
        void rotatePoints(const std::vector<Point2d>& pointsIn, double angle, std::vector<Point2d>& pointsOut);

        /** @brief Computes the corner score, position and orientation of point with given index. */
        double computeCornerScore(const std::vector<Point2d>& scan, int index, OC& keypoint);

        /** @brief Finds the neighborhood of point with the given index in vector. 
         * @param pointsIn vector of all the points 
         * @param index index of the center point in the 
         * @param neigh found neighborhood
         * @param midIndex index of the center point in neigh vector
         */
        void getNeighPoints(const std::vector<Point2d>& pointsIn, int index, std::vector<Point2d>& neigh, int& midIndex, double& dist) const;

        /** @brief Finds the peaks of score function using Non-Maxima Suppression (NMS) over a circular neighborhood with given radius.
         * It is almost a duplicate of FALKOExtractor similar function.
         */
        void NMSKeypoint(const std::vector<double>& scores, const LaserScan& scan, unsigned int ibeg, unsigned int iend, double radius, int minval, std::vector<int>& peaks);
    };
}
