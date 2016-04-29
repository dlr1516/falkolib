#include <iostream>
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
#include <falkolib/Feature/OC.h>
#include <falkolib/Feature/CGH.h>
#include <falkolib/Feature/BSC.h>

#include <falkolib/Feature/OCExtractor.h>
#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

#include <falkolib/Matching/NNMatcher.h>
#include <falkolib/Matching/CCDAMatcher.h>

#include "testData.h"

//#ifdef FEATURELIB_DEBUG
//#include <gnuplot-iostream.h>
//#endif

using namespace std;
using namespace falkolib;

int main(int argc, char** argv) {
    OCExtractor oe;
    oe.setAngleRes(M_PI / 180.0 * 0.25);
    oe.setTol(0.1);
    oe.setRangeRes(0.05);
    oe.setRangeMax(10.0);
    oe.setNMSRadius(0.3);
    oe.setNeighMinPoint(2);

    LaserScan scan1(-0.003316126, 2.0 * M_PI, 1440);
    scan1.fromRanges(testRangesOrtho1);
    LaserScan scan2(-0.003316126, 2.0 * M_PI, 1440);
    scan2.fromRanges(testRangesOrtho2);

    std::vector<OC> keypoints1;
    std::vector<OC> keypoints2;

    std::cout << "extract OC keypoints: " << std::endl;
    oe.extract(scan1, keypoints1);
    oe.extract(scan2, keypoints2);

    cout << "num keypoints1 extracted: " << keypoints1.size() << endl;
    for (auto& kp : keypoints1) {
        std::cout << kp.point.transpose() << ", index " << kp.index << ", radius " << kp.radius << ", orientation [deg] " << (180.0 / M_PI * kp.orientation) << std::endl;
    }

    cout << "num keypoints2 extracted: " << keypoints2.size() << endl;
    for (auto& kp : keypoints2) {
        std::cout << kp.point.transpose() << ", index " << kp.index << ", radius " << kp.radius << ", orientation [deg] " << (180.0 / M_PI * kp.orientation) << std::endl;
    }

//#ifdef FEATURELIB_DEBUG
//    Gnuplot gp("gnuplot -persist");
//    gp << "set size ratio -1\n";
//    for (int i = 0; i < scan1.points.size(); i += 10) {
//        gp << "set label \"" << i << "\" at " << scan1.points[i].x() << "," << scan1.points[i].y() << "\n";
//    }
//    gp << "plot '-' u 1:2 notitle pt 7 w p, '-' u 1:2 notitle pt 7 w p\n";
//    for (auto& p : scan1.points) {
//        gp << p.x() << " " << p.y() << "\n";
//    }
//    gp << "e" << std::endl;
//    for (auto& kp : keypoints1) {
//        gp << kp.point.x() << " " << kp.point.y() << "\n";
//    }
//    gp << "e" << std::endl;
//#endif

    CGHExtractor<OC> cgh(16);
    BSCExtractor<OC> bsc(16, 8);

    vector<CGH> cghDesc1;
    vector<CGH> cghDesc2;
    vector<BSC> bscDesc1;
    vector<BSC> bscDesc2;


    cgh.compute(scan1, keypoints1, cghDesc1);
    bsc.compute(scan1, keypoints1, bscDesc1);
    cgh.compute(scan2, keypoints2, cghDesc2);
    bsc.compute(scan2, keypoints2, bscDesc2);

    return 0;
}

