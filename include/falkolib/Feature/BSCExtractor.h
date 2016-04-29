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

#include <falkolib/Common/LaserScan.h>
#include <falkolib/Feature/Keypoint.h>
#include <falkolib/Feature/DescriptorExtractor.h>
#include <falkolib/Feature/BSC.h>

namespace falkolib {

	/**
	 * @brief class representing a BSC descriptor extractor engine 
	 */
	template <typename T>
	class BSCExtractor : public DescriptorExtractor<T, BSC> {
	public:

		/**
		 * @brief Constructor
		 * @param _circularSectorNumber number of grid circular sector
		 * @param _radialRingNumber number of grid radial number
		 * @param _useKeypointRadius if true, the selected neighborhood points search radius is keypoint one
		 * @param _radius neighborhood points search radius
		 */
		BSCExtractor(int _circularSectorNumber, int _radialRingNumber, bool _useKeypointRadius = true, double _radius = 0.1) {
			circularSectorNumber = _circularSectorNumber;
			radialRingNumber = _radialRingNumber;
			useKeypointRadius = _useKeypointRadius;
			radius = _radius;
		};

		/**
		 * @brief Extract BSC descriptor from a given scan and a list of keypoints
		 * @param scan input laser scan
		 * @param keypoints keypoints list
		 * @param descriptor extracted from scan and keypoints
		 */
		void compute(const LaserScan& scan, const std::vector<T>& keypoints, std::vector<BSC>& descriptors) {
			descriptors.reserve(keypoints.size());
			for (int i = 0; i < keypoints.size(); ++i) {
				std::vector<Point2d> neigh;
				int midIndex;
				if (useKeypointRadius) {
					scan.getNeighPoints(keypoints[i].index, keypoints[i].radius, neigh, midIndex);
					BSC desc(keypoints[i].radius, circularSectorNumber, radialRingNumber);
					desc.compute(neigh, midIndex);
					descriptors.push_back(std::move(desc));
				} else {
					scan.getNeighPoints(keypoints[i].index, radius, neigh, midIndex);
					BSC desc(radius, circularSectorNumber, radialRingNumber);
					desc.compute(neigh, midIndex);
					descriptors.push_back(std::move(desc));
				}
			}
		};

	private:
		int circularSectorNumber;
		int radialRingNumber;
		bool useKeypointRadius;
		double radius;
	};
}