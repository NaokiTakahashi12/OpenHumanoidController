
/**
  *
  * @file FootprintPlannerBase.hpp @brief Footprint base class @author Naoki
  * Takahashi
  *
  **/

#pragma once 

#include <memory>

#include "../Footprint/SpatialPoint.hpp"

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar = double>
		class FootprintPlannerBase {
			protected :
				static constexpr auto dimention_rank = 3;

				using SpatialPointPtr = typename Footprint::SpatialPoint<Scalar, dimention_rank>::Ptr;

			public :
				using Vector = Tools::Math::Vector<Scalar, dimention_rank>;
				using EulerAngles = Vector;

				using Ptr = std::unique_ptr<FootprintPlannerBase>;

				FootprintPlannerBase();
				virtual ~FootprintPlannerBase();

				FootprintPlannerBase &set_goal(const Vector &, const EulerAngles &);
				FootprintPlannerBase &set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g);
				FootprintPlannerBase &set_begin(const Vector &, const EulerAngles &);
				FootprintPlannerBase &set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g);

			protected :
				SpatialPointPtr begin_point, goal_point;
		};
	}
}

