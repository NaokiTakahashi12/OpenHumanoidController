
/**
  *
  * @file SpatialPoint.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SpatialPoint.hpp"

namespace FootStepPlanner {
	namespace Footprint {
		template struct SpatialPoint<float, 2>;
		template struct SpatialPoint<float, 3>;
		template struct SpatialPoint<double, 2>;
		template struct SpatialPoint<double, 3>;
	}
}
