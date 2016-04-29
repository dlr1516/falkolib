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

#include <falkolib/Common/Point.h>

namespace falkolib {

	/**
	 * @brief class representing a keypoint
	 * 
	 * Keypoints are defined as a 2d point
	 */
	class Keypoint {
	public:
		Point2d point;
		
		/**
		 * @brief measure distance between keypoints
		 * @param desc external keypoints
		 * @return measured distance
		 */
		inline double distance(const Keypoint& desc) const {
			return (point - desc.point).norm();
		}
	};
}