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

namespace falkolib {
	
	/**
	 * @brief class representing a simple feature matching engine 
	 */
	template <typename T>
	class Matcher {
	protected:
		
		/**
		 * @brief match features between two sets
		 * @param v1 first set of features
		 * @param v2 second set of features
		 * @param match matching vector representing associations, pair.first corresponds to v1 and pair.second corresponds to v2
		 * @return number of valid association in match
		 */
		virtual int match(const std::vector<T>& v1, const std::vector<T>& v2, std::vector<std::pair<int, int> >& match) = 0;
	};
}