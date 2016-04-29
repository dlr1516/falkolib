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
	 * @brief Binary Shape Context descriptor
	 * 
	 * This class represents a BSC Descriptor.
	 */
	class BSC : public Descriptor {
	public:
		
		/**
		 * @brief Constructor
		 * @param _radius descriptor max radius
		 * @param _circularSectorNumber number of circular sectors
		 * @param _radialRingNumber number of radial rings
		 * 
		 * Set the grid dimension and cells resolutions
		 */
		BSC(double _radius, int _circularSectorNumber, int _radialRingNumber);
		
		/**
		 * @brief Compute distance between two descriptors
		 * @param desc descriptor to measure distance
		 * @return the distance between *this and desc
		 * 
		 * Compute the distance between two descriptors of the same type (BSC)
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
		 * @brief Compute the grid descriptor
		 * @param neigh vector of neighborhood points
		 * @param centralPointIndex index of the central point in the neigh vector
		 */
		void compute(std::vector<Point2d>& neigh, int centralPointIndex);
		
	private:
		std::vector<std::vector<uint8_t> > grid;
		int circularSectorNumber;
		int radialRingNumber;
		double sectorResolution;
		double ringResolution;
		double radius;
		
		/** @brief compute the Hamming distance between two binary grid*/
		double HammingDistance(const std::vector<std::vector<uint8_t> >& g1, const std::vector<std::vector<uint8_t> >& g2) const;
	};
}