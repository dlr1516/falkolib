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
#include <falkolib/Feature/BSC.h>
#include <falkolib/Common/GeomUtils.h>
#include <limits>
#include <math.h>
#include <assert.h>
#include <algorithm>

using namespace std;

namespace falkolib {

	BSC::BSC(double _radius, int _circularSectorNumber, int _radialRingNumber) {
		radius = _radius;
		circularSectorNumber = _circularSectorNumber;
		sectorResolution = 2.0 * M_PI / circularSectorNumber;
		radialRingNumber = _radialRingNumber;
		ringResolution = radius / radialRingNumber;
	}

	void BSC::compute(std::vector<Point2d>& neigh, int centralPointIndex) {
		const int size = neigh.size();
		grid.resize(radialRingNumber, std::vector<uint8_t>(circularSectorNumber, 0));
		for (int i = 0; i < size; ++i) {
			if (i != centralPointIndex) {
				int col = static_cast<int> (floor((angleBetweenPoints(neigh[i], neigh[centralPointIndex]) + M_PI) / sectorResolution));
				assert(col < circularSectorNumber);
				int row = static_cast<int> (floor(((neigh[i] - neigh[centralPointIndex]).norm()) / ringResolution));
				assert(row < radialRingNumber);
				grid[row][col] = 1;
			}
		}
	}

	double BSC::distance(const Descriptor& desc) const{
		try {
			const BSC& d2 = dynamic_cast<const BSC&> (desc);
			assert(circularSectorNumber == d2.circularSectorNumber);
			assert(radialRingNumber == d2.radialRingNumber);
			if (circularSectorNumber == d2.circularSectorNumber && radialRingNumber == d2.radialRingNumber) {
				return HammingDistance(grid, d2.grid);
			}
		} catch (const std::bad_cast& e){
			;
		}
		return numeric_limits<double>::max();
	}

	double BSC::HammingDistance(const std::vector<std::vector<uint8_t> >& g1, const std::vector<std::vector<uint8_t> >& g2) const {
		double diff = 0.0;
		for (int i = 0; i < g1.size(); ++i) {
			for (int j = 0; j < g1[i].size(); ++j) {
				if (g1[i][j] != g2[i][j]) {
					diff++;
				}
			}
		}
		return diff;
	}

	void BSC::rotate(double theta) {
		int rot = static_cast<int> (floor((theta + M_PI) / sectorResolution)) % circularSectorNumber;
		for (auto& i : grid) {
			std::rotate(i.begin(), i.begin() + rot, i.end());
		}
	}
}