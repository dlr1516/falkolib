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
#include <falkolib/Feature/FALKO.h>
#include <falkolib/Common/GeomUtils.h>

namespace falkolib {

	/**
	 * @brief class representing a FALKO keypoint extractor engine 
	 */
	class FALKOExtractor : public KeypointExtractor<FALKO> {
	public:

		/**
		 * @brief Constructor
		 */
		FALKOExtractor();

		/**
		 * @brief Extract FALKO keypoints from a given scan 	 
		 * @param scan input laser scan
		 * @param features FALKO keypoint extracted from given scan
		 */
		void extract(const LaserScan& scan, std::vector<FALKO>& keypoints);

		/** @brief Set minimum score threshold for a candidate keypoint [%]*/
		inline void setMinScoreTh(double _minScoreTh) {
			minScoreTh = _minScoreTh;
		};

		/** @brief Set minimum extraction range [m]*/
		inline void setMinExtractionRange(double _minExtractionRange) {
			minExtractionRange = _minExtractionRange;
		};

		/** @brief Set maximum extraction range [m]*/
		inline void setMaxExtractionRange(double _maxExtractionRange) {
			maxExtractionRange = _maxExtractionRange;
		};

		/** @brief Enable subbeam accuracy in keypoint extraction */
		inline void enableSubbeam(bool _subbeam) {
			subbeam = _subbeam;
		};

		/** @brief Set Non-Maxima-Suppression radius [m]*/
		inline void setNMSRadius(double _NMSRadius) {
			NMSRadius = _NMSRadius;
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
		inline void setNeighMinPoint(double _neighMinPoint) {
			neighMinPoint = _neighMinPoint;
		};

		/** @brief Set b-ratio for geometric corner validation*/
		inline void setBRatio(double _bRatio) {
			bRatio = _bRatio;
		};

		/** @brief Set number of circular grid sector for score computation*/
		inline void setGridSectors(double _gridSectors) {
			gridSectors = _gridSectors;
		};

	private:
		double minScoreTh;
		double minExtractionRange;
		double maxExtractionRange;
		bool subbeam;
		double NMSRadius;
		double neighA;
		double neighB;
		int neighMinPoint;
		double bRatio;
		int gridSectors;

		/** @brief distance between grid circular sectors*/
		int circularSectorDistance(int a1, int a2, int res);

		/** @brief return index of corresponding circular sector in the score grid*/
		int getCircularSectorIndex(const Point2d& p, const Point2d& pmid, double theta);

		/** @brief return neighborhood search radius, different heuristics are used based on rho*/
		double getNeighRadius(double rho);

		/** @brief compute corner orientation given neighborhood points*/
		double getCornerOrientation(const std::vector<Point2d>& neigh, int midIndex);

		/** @brief Non-Maxima-Suppression function for keypoint extraction*/
		void NMSKeypoint(const std::vector<int>& scores, const LaserScan& scan, unsigned int ibeg, unsigned int iend, double radius, int minval, std::vector<int>& peaks);

		/** @brief compute corner position with subbeam accuracy*/
		void subBeamCorner(const LaserScan& scan, int index, double radius, Point2d& p);

		/** @brief generate a line model using least-square*/
		void generateLine(const std::vector<Point2d>& points, Eigen::Vector3d& model);

		/** @brief built-in 2x2 system solver function*/
		bool solveSystem2x2(double* A, double* b, double* x);
	};
}