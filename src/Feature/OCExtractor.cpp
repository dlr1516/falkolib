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
#include <falkolib/Feature/OCExtractor.h>
#include <cassert>

using namespace std;

namespace falkolib {

    OCExtractor::OCExtractor()
    //: houghSpectrum((M_PI / 180.0) * 0.5, 0.02, 10.0) {
    {
        tol = 0.1;
        angleRes = (M_PI / 180.0) * 0.5;
        rangeRes = 0.02;
        rangeMax = 10.0;
        nmsRadius = 0.1;
        neighA = 0.1;
        neighB = 0.07;
        neighMinPoint = 2;
        houghSpectrum.init(angleRes, rangeRes, rangeMax);
    }

    void OCExtractor::extract(const LaserScan& scan, std::vector<OC>& keypoints) {
        std::vector<Point2d> pointRot;
        std::vector<OC> keypointCandidates;
        std::vector<double> scores;
        std::vector<int> maxima;
        OC keypoint;
        double angle, score;
        int win;
        Eigen::Affine2d rot2orth, rot2inve;
        OC kp;

        // Computes dominant orientation angle and rotates the points
        angle = computeDominantAngle(scan.points);
        rotatePoints(scan.points, -angle, pointRot);
//        std::cout << "Rotated " << scan.points.size() << " by angle " << (180.0 / M_PI * angle) << " deg" << std::endl;
        rot2orth = Eigen::Affine2d::Identity();
        rot2orth.prerotate(angle);
        rot2inve = rot2orth.inverse();

        // For each point, computes the corresponding corner and score
        keypointCandidates.reserve(pointRot.size());
        scores.reserve(pointRot.size());
        for (int idx = 0; idx < pointRot.size(); ++idx) {
            score = computeCornerScore(pointRot, idx, keypoint);
            keypointCandidates.push_back(keypoint);
            scores.push_back(score);
        }

        // Extracts the maxima using non-maxima suppression (NMS)
        //        assert(scores.size() == scan.getNumBeams();
        NMSKeypoint(scores, scan, 0, scan.getNumBeams(), nmsRadius, 1.0, maxima);

        keypoints.clear();
        keypoints.reserve(maxima.size());
        for (auto& idx : maxima) {
            kp = keypointCandidates[idx];
            kp.point = rot2orth * keypointCandidates[idx].point;
            double a = keypointCandidates[idx].orientation + angle;
            kp.orientation = atan2(sin(a), cos(a));
            keypoints.push_back(kp);
        }
    }

    void OCExtractor::rotatePoints(const std::vector<Point2d>& pointsIn, double angle, std::vector<Point2d>& pointsOut) {
        Point2d p;
        double ct = cos(angle);
        double st = sin(angle);

        pointsOut.clear();
        pointsOut.reserve(pointsIn.size());
        for (int i = 0; i < pointsIn.size(); ++i) {
            p.x() = pointsIn[i].x() * ct - pointsIn[i].y() * st;
            p.y() = pointsIn[i].x() * st + pointsIn[i].y() * ct;
            pointsOut.push_back(p);
        }
    }

    double OCExtractor::computeDominantAngle(const std::vector<Point2d>& points) {
        int maxIdx;
        double angle;

        houghSpectrum.insertPoint(points.begin(), points.end());
        houghSpectrum.orthoSpectrum().maxCoeff(&maxIdx);
        angle = angleRes * maxIdx;
        return angle;
    }

    double OCExtractor::computeCornerScore(const std::vector<Point2d>& pointsIn, int index, OC& keypoint) {
        vector<Point2d> neigh;
        double range, distNeigh;
        double score, dx, dy;
        int midIndex, orientationMax;
        int neighSize, neighSizeL, neighSizeR;
        int xnum, ynum;
        int orientationBin[4] = {0, 0, 0, 0};

        // Computes the neighborhood
        neigh.clear();
        getNeighPoints(pointsIn, index, neigh, midIndex, distNeigh);

        // Check if it current point is in the center of the neighbor interval
        neighSize = neigh.size();
        neighSizeL = midIndex;
        neighSizeR = neighSize - midIndex - 1;
        //        std::cout << __FILE__ << "," << __LINE__ << ": distNeigh " << distNeigh << ", neighSize " << neighSize << ", neighSizeL " << neighSizeL << ", neighSizeR " << neighSizeR << std::endl;
        if (neighSizeL < neighMinPoint || neighSizeR < neighMinPoint) {
            //            std::cout << "  unbalanced neighborhood: distNeigh " << distNeigh << ", neighSize " << neighSize << ", neighSizeL " << neighSizeL << ", neighSizeR " << neighSizeR << ", neighMinPoint " << neighMinPoint << std::endl;
            return 0.0;
        }

        // Check that the neighborhood has a triangular shape rather than a flat one
        //        double triangleBLength = pointsDistance(neigh.front(), neigh.back());
        //        double triangleHLength = std::abs(signedTriangleArea(neigh[midIndex], neigh.front(), neigh.back())) / triangleBLength;
        //        std::cout << __FILE__ << "," << __LINE__ << ": triangleBLength " << triangleBLength << " > " << (distNeigh / bRatio) << ", triangleHLength " << triangleHLength << ", (distNeigh / bRatio) " << (distNeigh / bRatio) << std::endl;
        //        if (triangleBLength < (distNeigh / bRatio) || triangleHLength < (distNeigh / bRatio)) {
        //            return 0.0;
        //        }

        // Computes the keypoint/corner 
        keypoint.index = index;
        keypoint.point.x() = 0.0;
        keypoint.point.y() = 0.0;
        keypoint.index = index;
        keypoint.radius = distNeigh;
        xnum = 0;
        ynum = 0;
        for (auto& np : neigh) {
            dx = np.x() - pointsIn[index].x();
            dy = np.y() - pointsIn[index].y();
            // a. Neighboor pointIn[i] has similar x coordinate and different y coordinate.
            //    Condition std::abs(dy) < distNeigh means that is inside circle with radius distNeigh.
            // b. Neighboor pointIn[i] has similar y coordinate and different x coordinate.
            //    Condition std::abs(dx) < distNeigh means that is inside circle with radius distNeigh.
            if (std::abs(dx) < tol && tol < std::abs(dy) && std::abs(dy) < distNeigh) {
                xnum++;
                keypoint.point.x() += (np.x() - keypoint.point.x()) / xnum;
                if (dy > 0) {
                    orientationBin[NE]++; // NE
                    orientationBin[NW]++; // NW
                } else {
                    orientationBin[SW]++; // SW
                    orientationBin[SE]++; // SE
                }
            }
            if (std::abs(dy) < tol && tol < std::abs(dx) && std::abs(dx) < distNeigh) {
                ynum++;
                keypoint.point.y() += (np.y() - keypoint.point.y()) / ynum;
                if (dx > 0) {
                    orientationBin[NE]++; // NE
                    orientationBin[SE]++; // SE
                } else {
                    orientationBin[NW]++; // NW
                    orientationBin[SW]++; // SW
                }
            }
            //            std::cout << "  index " << index << " dx " << dx << ", dy " << dy << ", xnum " << xnum << ", ynum " << ynum << "\n";
        }
        if (xnum == 0) {
            keypoint.point.x() = pointsIn[index].x();
        }
        if (ynum == 0) {
            keypoint.point.y() = pointsIn[index].y();
        }

        score = 1.0 * (xnum + ynum) / (0.1 + std::abs(xnum - ynum));
//        if (score > 1.0 || index == 1197 || (1304 <= index && index <= 1305) || (117 <= index && index <= 119)) {
//            std::cout << __FILE__ << "," << __LINE__ << "\n  index " << index << ", score " << score << ", xnum " << xnum << ", ynum " << ynum
//                    << "\n  neighSize " << neighSize << ", neighSizeL " << neighSizeL << ", neighSizeR " << neighSizeR << ", neighMinPoint " << neighMinPoint << std::endl;
//        }

        // Computes keypoint orientation according to maximum of orientationBin[] histogram:
        // * orientationBin[0]: I   quadrant, 45 deg
        // * orientationBin[1]: II  quadrant, 135 deg
        // * orientationBin[2]: III quadrant, 225 deg (or -135 deg)
        // * orientationBin[3]: IV  quadrant, 315 deg (or -45 deg)
        orientationMax = 0;
        for (int i = 1; i < 4; ++i) {
            if (orientationBin[i] > orientationBin[orientationMax]) {
                orientationMax = i;
            }
        }
        keypoint.orientation = M_PI * (0.25 + 0.5 * orientationMax);
        keypoint.orientation = atan2(sin(keypoint.orientation), cos(keypoint.orientation));
        return score;
    }

    void OCExtractor::getNeighPoints(const std::vector<Point2d>& pointsIn, int index, std::vector<Point2d>& neigh, int& midIndex, double& dist) const {
        double range;
        int imin, imax, win;

        neigh.clear();
        range = pointsIn[index].norm();
        dist = neighA * exp(neighB * range);
        if (range > dist) {
            win = (int) ceil(asin(dist / range) / angleRes);
        } else {
            win = pointsIn.size();
        }
        imin = std::max(index - win, 0);
        imax = std::min(index + win + 1, (int) pointsIn.size());
        for (int i = imin; i < imax; ++i) {
            if ((pointsIn[i] - pointsIn[index]).norm() < dist) {
                neigh.push_back(pointsIn[i]);
                if (i == index) {
                    midIndex = neigh.size();
                }
            }
        }
    }

    void OCExtractor::NMSKeypoint(const std::vector<double>& scores, const LaserScan& scan, unsigned int ibeg, unsigned int iend, double radius, int minval, std::vector<int>& peaks) {
        unsigned i, imax;
        unsigned j, jbeg, jend, jmax;
        std::vector<int> candidates;
        peaks.clear();

        i = ibeg;
        imax = ibeg;
        while (i < iend) {
            int win;
            if (radius >= scan.ranges[i]) {
                win = std::floor(std::asin(0.8) / scan.getAngleInc());
            } else {
                win = std::floor(std::asin(radius / scan.ranges[i]) / scan.getAngleInc());
            }

            jmax = i;

            if (imax + win < i) {
                jbeg = (i >= win ? i - win : 0);
                imax = i;
            } else {
                jbeg = i;
            }

            jend = (i + win + 1 < iend ? i + win + 1 : iend);
            for (j = jbeg; j < jend; ++j) {
                if (scores[j] > scores[jmax]) {
                    jmax = j;
                }
            }
            imax = (scores[jmax] >= scores[imax] ? jmax : imax);

            if (i == imax && scores[i] > minval) {
                candidates.push_back(i);
            }
            if (jmax > i) i = jmax;
            else ++i;
        }

        int i1 = 0;
        int i2 = 0;
        int counter = 0;

        for (i1 = 0, i2 = 0; i1 < candidates.size(); ++i1) {
            if (scores[candidates[i2]] == scores[candidates[i1]]) {
                counter++;
                if (2 * abs(i2 - i1) > counter) {
                    ++i2;
                }
            } else {
                peaks.push_back(candidates[i2]);
                i2 = i1;
                counter = 0;
            }
        }
        if (i2 != candidates.size()) {
            peaks.push_back(candidates[i2]);
        }
    }

}

