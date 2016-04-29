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
	 * @brief class representing a Affine Hough Transform feature matching engine 
	 */
	template <typename T = Keypoint, typename D = Descriptor>
	class AHTMatcher : public Matcher<T> {
	public:

		/**
		 * @brief Constructor
		 */
		AHTMatcher() : AHTMatcher(0.1, 0.1, 0.04, 5, 5, 1.57) {
		};

		/**
		 * @brief Constructor
		 */
		AHTMatcher(double _xRes, double _yRes, double _thetaRes, double _xAbsMax, double _yAbsMax, double _thetaAbsMax) {
			xRes = _xRes;
			yRes = _yRes;
			thetaRes = _thetaRes;
			xAbsMax = _xAbsMax;
			yAbsMax = _yAbsMax;
			thetaAbsMax = _thetaAbsMax;
			xSize = static_cast<int> (2.0 * xAbsMax / xRes);
			ySize = static_cast<int> (2.0 * yAbsMax / yRes);
			thetaSize = static_cast<int> (2.0 * thetaAbsMax / thetaRes);
			matchesGrid.resize(xSize * ySize * thetaSize);
			txMax = 0;
			tyMax = 0;
			thetaMax = 0;
		};

		/**
		 * @brief match keypoints between two sets using AHT matcher
		 * @param v1 first set of keypoints
		 * @param v2 second set of keypoints
		 * @param match matching vector representing associations, pair.first corresponds to v1 and pair.second corresponds to v2
		 * @return number of valid association in match
		 */
		int match(const std::vector<T>& v1, const std::vector<T>& v2, std::vector<std::pair<int, int> >& match) {
			std::vector<std::pair<int, int> > assoInit;
			for (int i1 = 0; i1 < v1.size(); ++i1) {
				for (int i2 = 0; i2 < v2.size(); ++i2) {
					if (v1[i1].distance(v2[i2]) < distTh) {
						assoInit.push_back(std::make_pair(i1, i2));
					}
				}
			}

			return getBestMatching(v1, v2, assoInit, match);
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
			std::vector<std::pair<int, int> > assoInit;
			for (int i1 = 0; i1 < keypoints1.size(); ++i1) {
				for (int i2 = 0; i2 < keypoints2.size(); ++i2) {
					if (keypoints1[i1].distance(keypoints2[i2]) < distTh && descriptors1[i1].distance(descriptors2[i2]) < descTh) {
						assoInit.push_back(std::make_pair(i1, i2));
					}
				}
			}

			return getBestMatching(keypoints1, keypoints2, assoInit, match);
		}

		/** @brief set euclidean distance threshold for keypoints distance measurement*/
		void setDistanceThreshold(double _th) {
			distTh = _th;
		}

		/** @brief set descriptor threshold for distance measurement*/
		void setDescriptorThreshold(double _th) {
			descTh = _th;
		}



	private:
		double distTh;
		double descTh;
		double xRes;
		double yRes;
		double thetaRes;
		double xAbsMax;
		double yAbsMax;
		double thetaAbsMax;
		int xSize;
		int ySize;
		int thetaSize;
		std::vector<std::vector<std::pair<int, int> > > matchesGrid;
		//		std::vector<int> countGrid;
		int txMax;
		int tyMax;
		int thetaMax;

		/** @brief get grid index from separated transform indexes*/
		int getGridIndex(int ix, int iy, int it) {
			return ix + iy * xSize + it * xSize * ySize;
		}

		/** @brief get best matching given point and an initial guest associations list*/
		int getBestMatching(const std::vector<T>& v1, const std::vector<T>& v2, const std::vector<std::pair<int, int> >& init, std::vector<std::pair<int, int> >& match) {
			for (auto& asso : init) {
				for (int it = 0; it < thetaSize; ++it) {
					double theta = thetaRes * (it - thetaSize / 2);
					const Point2d& p1 = v1[asso.first].point;
					const Point2d& p2 = v2[asso.second].point;
					double tx = p1[0] - p2[0] * std::cos(theta) + p2[1] * std::sin(theta);
					double ty = p1[1] - p2[0] * std::sin(theta) - p2[1] * std::cos(theta);

					int ix = static_cast<int> (tx / xRes + xSize / 2);
					int iy = static_cast<int> (ty / yRes + ySize / 2);

					if (ix >= 0 && ix < xSize && iy >= 0 && iy < ySize && it >= 0 && it < thetaSize) {
//						std::cout << "ind: " << getGridIndex(ix, iy, it) << "\t" << ix << "\t" << iy << "\t" << it << "\tpoints: " << p1.transpose() << "\t" << p2.transpose() << std::endl;
						matchesGrid[getGridIndex(ix, iy, it)].push_back(asso);

						if (matchesGrid[getGridIndex(ix, iy, it)].size() >= matchesGrid[getGridIndex(txMax, tyMax, thetaMax)].size()) {
							txMax = ix;
							tyMax = iy;
							thetaMax = it;
						}
					}
				}
			}

			match = matchesGrid[getGridIndex(txMax, tyMax, thetaMax)];
			int numAsso = match.size();
			for (int i = 0; i < v1.size(); ++i) {
				bool found = false;
				for (int j = 0; j < match.size(); ++j) {
					if (i == match[j].first) {
						found = true;
						break;
					}
				}
				if (not found) {
					match.push_back(std::make_pair(i, -1));
				}
			}
			return numAsso;
		}
	};
}