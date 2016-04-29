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

#include <falkolib/Matching/Matcher.h>

#include <vector>
#include <falkolib/Feature/Descriptor.h>
#include <falkolib/Feature/Keypoint.h>

namespace falkolib {

	/**
	 * @brief class representing a simple Nearest-Neighborhood feature matching engine 
	 */
	template <typename T = Keypoint, typename D = Descriptor>
	class NNMatcher : public Matcher<T> {
	public:

		/**
		 * @brief Constructor
		 */
		NNMatcher() {

		};

		/**
		 * @brief match keypoints or descriptors between two sets
		 * @param v1 first set of keypoints or descriptors
		 * @param v2 second set of keypoints or descriptors
		 * @param match matching vector representing associations, pair.first corresponds to v1 and pair.second corresponds to v2
		 * @return number of valid association in match
		 */
		int match(const std::vector<T>& v1, const std::vector<T>& v2, std::vector<std::pair<int, int> >& match) {
			match.clear();
			int imin;
			double dmin, d;
			int counter = 0;
			std::vector<bool> matched(v2.size(), false);
			for (int i1 = 0; i1 < (int) v1.size(); ++i1) {
				imin = -1;
				dmin = 1.05 * distTh;
				for (int i2 = 0; i2 < (int) v2.size(); ++i2) {
					if (not matched[i2]) {
						d = v1[i1].distance(v2[i2]);
						if (d < dmin) {
							imin = i2;
							dmin = d;
						}
					}
				}
				if (dmin < distTh) {
					match.push_back(std::make_pair(i1, imin));
					matched[imin] = true;
					counter++;
				} else {
					match.push_back(std::make_pair(i1, -1));
				}
			}

			return counter;
		}

		/**
		 * @brief match keypoints between two sets using descriptors
		 * @param keypoints1 first set of keypoints
		 * @param descriptors1 first set of descriptors
		 * @param keypoints2 second set of keypoints
		 * @param descriptors2 second set of descriptors
		 * @param match matching vector representing associations, pair.first corresponds to v1 and pair.second corresponds to v2
		 * @return number of valid association in match
		 */
		int match(const std::vector<T>& keypoints1, const std::vector<D>& descriptors1, const std::vector<T>& keypoints2, const std::vector<D>& descriptors2, std::vector<std::pair<int, int> >& match) {
			match.clear();
			int imin;
			double dmin, d, ddesc;
			int counter = 0;
			std::vector<bool> matched(keypoints2.size(), false);
			for (int i1 = 0; i1 < (int) keypoints1.size(); ++i1) {
				imin = -1;
				dmin = 1.05 * distTh;
				for (int i2 = 0; i2 < (int) keypoints2.size(); ++i2) {
					if (not matched[i2]) {
						d = keypoints1[i1].distance(keypoints2[i2]);
						ddesc = descriptors1[i1].distance(descriptors2[i2]);
						if (d < dmin && ddesc < descTh) {
							imin = i2;
							dmin = d;
						}
					}
				}
				if (dmin < distTh && descriptors1[i1].distance(descriptors2[imin]) < descTh) {
					match.push_back(std::make_pair(i1, imin));
					matched[imin] = true;
					counter++;
				} else {
					match.push_back(std::make_pair(i1, -1));
				}
			}

			return counter;
		}

		/** @brief set euclidean distance threshold for keypoints distance measurements*/
		void setDistanceThreshold(double _th) {
			distTh = _th;
		}

		/** #brief set descriptor threshold for distance measurements*/
		void setDescriptorThreshold(double _th) {
			descTh = _th;
		}

	private:
		double distTh;
		double descTh;

	};
}