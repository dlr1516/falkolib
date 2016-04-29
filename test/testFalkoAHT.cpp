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
#include <iostream>


#include <falkolib/Feature/FALKO.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Feature/FALKOExtractor.h>

#include <falkolib/Feature/BSCExtractor.h>

#include <falkolib/Matching/NNMatcher.h>
#include <falkolib/Matching/AHTMatcher.h>

#include "testData.h"

using namespace std;
using namespace falkolib;

int main(int argc, char** argv) {
	LaserScan scan1(-0.003316126, 2.0 * M_PI, 1440);
	scan1.fromRanges(testRanges);
	LaserScan scan2(-0.003316126, 2.0 * M_PI, 1440);
	scan2.fromRanges(testRanges2);

	FALKOExtractor fe;
	fe.setMinExtractionRange(0);
	fe.setMaxExtractionRange(30);
	fe.enableSubbeam(true);
	fe.setNMSRadius(0.1);
	fe.setNeighB(0.07);
	fe.setBRatio(2.5);
	fe.setGridSectors(16);

	std::vector<FALKO> keypoints1;
	std::vector<FALKO> keypoints2;


	fe.extract(scan1, keypoints1);
	fe.extract(scan2, keypoints2);

	cout << "num keypoints1 extracted: " << keypoints1.size() << endl;
	cout << "num keypoints2 extracted: " << keypoints2.size() << endl;

	BSCExtractor<FALKO> bsc(16, 8);
	vector<BSC> bscDesc1;
	vector<BSC> bscDesc2;


	bsc.compute(scan1, keypoints1, bscDesc1);
	bsc.compute(scan2, keypoints2, bscDesc2);




	NNMatcher<FALKO> matcher;
	matcher.setDistanceThreshold(0.1);
	std::vector<std::pair<int, int> > assoNN;
	std::cout << "num matching NN: " << matcher.match(keypoints1, keypoints2, assoNN) << endl;
	for (auto& match : assoNN) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << endl;
		}
	}

	cout << endl;
	AHTMatcher<FALKO> aht(0.01, 0.01, 0.001, 0.2, 0.2, 0.05);
	aht.setDistanceThreshold(0.1);
	std::vector<std::pair<int, int> > assoAHT;
	std::cout << "num matching AHT: " << aht.match(keypoints1, keypoints2, assoAHT) << endl;
	for (auto& match : assoAHT) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (keypoints1[i1].distance(keypoints2[i2])) << endl;
		}
	}
	
	Eigen::Affine2d transformNN;
	Eigen::Affine2d transformAHT;
	computeTransform(keypoints1, keypoints2, assoNN, transformNN);
	computeTransform(keypoints1, keypoints2, assoAHT, transformAHT);
	
	std::cout << "NN transform: " << std::endl;
	std::cout << transformNN.inverse().matrix() << std::endl;
	std::cout << "AHT transform: " << std::endl;
	std::cout << transformAHT.inverse().matrix() << std::endl;

	return 0;
}

