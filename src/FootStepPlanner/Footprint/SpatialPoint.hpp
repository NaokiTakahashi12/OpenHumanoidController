
/**
  *
  * @file SpatialPoint.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>

#include <Tools/Math/Matrix.hpp>

namespace FootStepPlanner {
	namespace Footprint {
		template <typename Scalar, int Dimention>
		struct SpatialPoint {
			static constexpr auto rank = Dimention;

			using Vector = Tools::Math::Vector<Scalar, rank>;
			using EulerAngles = Vector;
			using Ptr = std::unique_ptr<SpatialPoint>;

			Vector centor;
			EulerAngles centor_eular_angles;

			static Ptr make_ptr();
		};
	}
}

