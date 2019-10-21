
/**
  *
  * @file Bipedal.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SpatialPoint.hpp"

namespace FootStepPlanner {
	namespace Footprint {
		template <typename Scalar>
		struct Bipedal : SpatialPoint<Scalar, 3> {
			using SpatialPoint<Scalar, 3>::rank;
			using typename SpatialPoint<Scalar, rank>::Vector;
			using typename SpatialPoint<Scalar, rank>::EularAngles;
			using Ptr = std::unique_ptr<Bipedal>;

			Vector left,
				   right;

			EularAngles left_eular_angles,
						right_eular_angles;
		};
	}
}
