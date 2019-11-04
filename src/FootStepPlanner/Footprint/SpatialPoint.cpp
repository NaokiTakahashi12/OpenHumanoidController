
/**
  *
  * @file SpatialPoint.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SpatialPoint.hpp"

namespace FootStepPlanner {
	namespace Footprint {
		template <typename Scalar, int Dimention>
		typename SpatialPoint<Scalar, Dimention>::Ptr SpatialPoint<Scalar, Dimention>::make_ptr() {
			return std::make_unique<SpatialPoint>();
		}

		template struct SpatialPoint<float, 2>;
		template struct SpatialPoint<float, 3>;
		template struct SpatialPoint<double, 2>;
		template struct SpatialPoint<double, 3>;
	}
}
