
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
			using typename SpatialPoint<Scalar, rank>::EulerAngles;
			using Ptr = std::unique_ptr<Bipedal>;

			Vector left,
				   right;

			EulerAngles left_eular_angles,
						right_eular_angles;

			static Ptr make_ptr();
		};
	}
}
