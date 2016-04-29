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

/**
 * @brief Useful geometric functions
 */

#include <falkolib/Common/Point.h>

#include <Eigen/Dense>

namespace falkolib {

    /**
     * @brief point-to-point distance
     * @param p1 point 1
     * @param p2 point 2
     * @return distance between p1 and p2F
     */
    template <typename T>
    double pointsDistance(const T& p1, const T& p2) {
        return (p2 - p1).norm();
    }

    /**
     * @brief angle between two given points
     * @param p1 point 1
     * @param p2 point 2
     * @return angle between p1 and p2 [rad]
     */
    template <typename T>
    double angleBetweenPoints(const T& p1, const T& p2) {
        double angle = atan2(p2[1] - p1[1], p2[0] - p1[0]);
        //numeric problems..........
        if (angle >= M_PI) return M_PI - 0.000001;
        if (angle <= -M_PI) return -M_PI + 0.000001;
        return angle;
    }

    /**
     * @brief This function computes the inner area of a triangle defined by 3 vertices
     * @param p0 triangle first vertex
     * @param p1 triangle second vertex
     * @param p2 triangle third vertex
     * @return signed area of the inner triangle, p0-p1 as base of the triangle
     */
    template <typename T>
    double signedTriangleArea(const T& p0, const T& p1, const T& p2) {
        return ((p2[1] - p1[1]) * p0[0] - (p2[0] - p1[0]) * p0[1] + p2[0] * p1[1] - p2[1] * p1[0]);
    }

    /**
     * @brief Compute Affine transform between two points sets using least square regression
     * @param v1 first points set
     * @param v2 second points set
     * @param indices matching vectors, pair.first corresponds to v1 and pair.second corresponds to v2
     * @param transform resulting affine transform which move v2 in v1 frame.
     * @return if false, the resulting transform is invalid
     */
    template <typename T>
    bool computeTransform(const std::vector<T>& v1, const std::vector<T>& v2, const std::vector<std::pair<int, int> >& indices, Eigen::Affine2d& transform) {

        Eigen::Vector2d t1 = Eigen::Vector2d::Zero();
        Eigen::Vector2d t2 = Eigen::Vector2d::Zero();
        Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
        int n = 0;
        for (int i = 0; i < (int) indices.size(); ++i) {
            if (0 <= indices[i].first && indices[i].first < (int) v1.size() &&
                    0 <= indices[i].second && indices[i].second < (int) v2.size()) {
                t1 += v1[indices[i].first].point;
                t2 += v2[indices[i].second].point;
                n++;
            }
        }
        if (n == 0) {
            return false;
        }
        t1 = (1.0 / n) * t1;
        t2 = (1.0 / n) * t2;
        for (int i = 0; i < (int) indices.size(); ++i) {
            if (0 <= indices[i].first && indices[i].first < (int) v1.size() &&
                    0 <= indices[i].second && indices[i].second < (int) v2.size()) {
                S += (v2[indices[i].second].point - t2) * ((v1[indices[i].first].point - t1).transpose());
            }
        }
        double theta = std::atan2(S(0, 1) - S(1, 0), S(0, 0) + S(1, 1));
        Eigen::Rotation2Dd rot(theta);
        Eigen::Vector2d transl = t1 - (rot * t2);
        transform = Eigen::Affine2d::Identity();
        transform.prerotate(rot);
        transform.pretranslate(transl);

        return true;
    }

}