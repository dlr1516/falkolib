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

#include <falkolib/Feature/Descriptor.h>
#include <falkolib/Common/Point.h>
#include <vector>

namespace falkolib {

	/**
	 * @brief Cumulative Gaussian Histogram descriptor
	 * 
	 * This class represents a CGH Descriptor.
	 */
	class CGH : public Descriptor {
	public:
		
		/**
		 * @brief Constructor
		 * @param _radius descriptor max radius
		 * @param _circularSectorNumber number of histogram bins
		 * 
		 * Set the histogram dimension and the circular sectors resolutions
		 */
		CGH(double _radius, int _circularSectorNumber);
		
		/**
		 * @brief Compute distance between two descriptors
		 * @param desc descriptor to measure distance
		 * @return the distance between *this and desc
		 * 
		 * Compute the distance between two descriptors of the same type (CGH)
		 */
		double distance(const Descriptor& desc) const;
		
		/**
		 * @brief Rotate the descriptor grid
		 * @param theta angle of rotation [rad]
		 * 
		 * Rotate the descriptor grid of a number of circular sector based on theta
		 */
		void rotate(double theta);
		
		/**
		 * @brief Compute the histogram descriptor
		 * @param neigh vector of neighborhood points
		 * @param centralPointIndex index of the central point in the neigh vector
		 */
		void compute(std::vector<Point2d>& neigh, int centralPointIndex);
		
	private:
		std::vector<double> histogram;
		int circularSectorNumber;
		double sectorResolution;
		double radius;
		
		/** @brief compute the Chi-squared distance between two histograms*/
		double SymmetricChiSquaredDistance(const std::vector<double>& h1, const std::vector<double>& h2) const;
	};
}