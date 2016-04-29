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
#include <falkolib/Feature/CGH.h>
#include <falkolib/Common/GeomUtils.h>
#include <limits>
#include <math.h>
#include <assert.h>
#include <algorithm>

#include <iostream>

using namespace std;

namespace falkolib {

	CGH::CGH(double _radius, int _circularSectorNumber) {
		radius = _radius;
		circularSectorNumber = _circularSectorNumber;
		sectorResolution = 2.0 * M_PI / circularSectorNumber;
	}

	void CGH::compute(std::vector<Point2d>& neigh, int centralPointIndex) {
		const int size = neigh.size();
		histogram.resize(circularSectorNumber, 0);
		for (int i = 0; i < size; ++i) {
			if (i != centralPointIndex) {
				int ind = static_cast<int> (floor((angleBetweenPoints(neigh[i], neigh[centralPointIndex]) + M_PI) / sectorResolution));
				assert(ind < circularSectorNumber);
				if (ind == 0) {
					histogram[circularSectorNumber - 1]++;
				} else {
					histogram[ind - 1]++;
				}
				histogram[ind] += 5;
				if (ind == (circularSectorNumber - 1)) {
					histogram[0]++;
				} else {
					histogram[ind + 1]++;
				}
			}
		}
		double sum = 0;
		for (auto& l : histogram) {
			sum += l;
		}
		for (auto& l : histogram) {
			l /= sum;
		}
	}

	double CGH::distance(const Descriptor& desc) const {
		try {
			const CGH& d2 = dynamic_cast<const CGH&> (desc);
			assert(circularSectorNumber == d2.circularSectorNumber);
			if (circularSectorNumber == d2.circularSectorNumber) {
				return SymmetricChiSquaredDistance(histogram, d2.histogram);
			}
		} catch (const std::bad_cast& e){
			;
		}
		return numeric_limits<double>::max();
	}

	double CGH::SymmetricChiSquaredDistance(const std::vector<double>& h1, const std::vector<double>& h2) const {
		double sum1 = 0.0;
		for (int i = 0; i < h1.size(); ++i) {
			sum1 += ((h1[i] - h2[i])*(h1[i] - h2[i]) / (h1[i] + h2[i] + 0.00000001));
		}
		return sum1/2.0;
	}

	void CGH::rotate(double theta) {
		int rot = (static_cast<int> (floor((theta + M_PI) / sectorResolution)) % circularSectorNumber);
		std::rotate(histogram.begin(), histogram.begin()+rot, histogram.end());
	}
}