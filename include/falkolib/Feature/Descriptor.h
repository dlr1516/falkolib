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

namespace falkolib {

	/**
	 * @brief class representing a generic descriptor
	 * 
	 * No properties are defined for a generic descriptor
	 */

	class Descriptor {
	protected:

		/**
		 * @brief Compute distance between two descriptors
		 * @param desc descriptor to measure distance
		 * @return the distance between *this and desc
		 */
		virtual double distance(const Descriptor& desc) const = 0;
	};
}