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

#include <vector>
#include <falkolib/Common/Point.h>
#include <falkolib/Common/GeomUtils.h>

namespace falkolib {

	/**
	 * @brief Laser scan container
	 * 
	 * This class provides an essential interface for laser scan data 
	 */
	class LaserScan {
	public:

		/**
		 * @brief Constructor
		 */
		LaserScan() {
			angleMin = 0;
			fov = 0;
			angleInc = 0;
			numBeams = 0;
			timestamp = 0;
		}

		/**
		 * @brief Constructor
		 * @param _angleMin laser scanner start angle [rad]
		 * @param _fov laser scanner field of view [rad]
		 * @param _numBeams laser scanner number of beams
		 */
		LaserScan(double _angleMin, double _fov, int _numBeams) {
			angleMin = _angleMin;
			fov = _fov;
			angleInc = _fov / _numBeams;
			numBeams = _numBeams;
			timestamp = 0;
		}

		/** @brief Set laser scanner start angle [rad] */
		inline void setAngleMin(double _angleMin) {
			angleMin = _angleMin;
		};

		/** @brief Set laser scanner field of view [rad] */
		inline void setLaserFoV(double _fov) {
			fov = _fov;
		};

		/** @brief Set laser scanner angle increment [rad] */
		inline void setAngleInc(double _angleInc) {
			angleInc = _angleInc;
		};

		/** @brief Set laser scanner number of beams */
		inline void setNumBeams(int _numBeams) {
			numBeams = _numBeams;
		};

		/** @brief Set scan beginning timestamp [s] */
		inline void setTimestamp(double _timestamp) {
			timestamp = _timestamp;
		};

		/** @brief Get laser scanner number of beams */
		inline int getNumBeams() const {
			return numBeams;
		};

		/** @brief Get laser scanner angle increment [rad] */
		inline double getAngleInc() const {
			return angleInc;
		};

		/**
		 * @brief Compute scan points from ranges
		 * 
		 * @param _ranges plain array of double representing the scan ranges
		 */
		inline void fromRanges(const double* _ranges) {
			fromRanges(std::vector<double>(_ranges, _ranges + numBeams));
		}

		/**
		 * @brief Compute scan points from ranges
		 * 
		 * @param _ranges std::vector of double representing the scan ranges
		 */
		inline void fromRanges(const std::vector<double>& _ranges) {
			double theta;
			ranges = _ranges;
			points.resize(numBeams);
			for (int i = 0; i < numBeams; ++i) {
				theta = i * angleInc + angleMin;
				points[i][0] = ranges[i] * std::cos(theta);
				points[i][1] = ranges[i] * std::sin(theta);
			}
		}

		/**
		 * @brief compute neighborhood points list given a single point index and a search radius
		 * @param candIndex index of the central point
		 * @param radius search euclidean radius [m]
		 * @param neigh vector of the neighborhood points
		 * @param midIndex index representing the central point in the neigh vector
		 */
		void getNeighPoints(int candIndex, double radius, std::vector<Point2d>& neigh, int& midIndex) const {
			const Point2d& candPoint = points[candIndex];
			int alpha = std::floor(std::asin(radius / ranges[candIndex]) / angleInc);
			int begIndex = std::max(0, candIndex - alpha);
			int endIndex = std::min(candIndex + alpha + 1, numBeams);
			for (int i = begIndex; i <= endIndex; ++i) {
				if (pointsDistance(points[i], candPoint) <= radius) {
					if (i == candIndex) {
						midIndex = neigh.size();
					}
					neigh.push_back(points[i]);
				}
			}
		}


		std::vector<double> ranges;
		std::vector<Point2d> points;

	private:

		double angleMin;
		double fov;
		double angleInc;
		int numBeams;
		double timestamp;
	};
}