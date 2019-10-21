
/**
  *
  * @file FootprintPlannerBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "FootprintPlannerBase.hpp"

#include <cmath>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		FootprintPlannerBase<Scalar>::FootprintPlannerBase() {
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar>::~FootprintPlannerBase() {
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_goal(const Vector &point, const EulerAngles &eularangles) {
			if(!goal_point) {
				goal_point = std::make_unique<Footprint::SpatialPoint<Scalar, dimention_rank>>();
			}
			goal_point->centor = point;
			goal_point->centor_eular_angles = eularangles;
			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g) {
			if(!goal_point) {
				goal_point = std::make_unique<Footprint::SpatialPoint<Scalar, dimention_rank>>();
			}
			goal_point->centor << x, y, z;
			goal_point->centor_eular_angles << a, b, g;
			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_begin(const Vector &point, const EulerAngles &eularangles) {
			if(!begin_point) {
				begin_point = std::make_unique<Footprint::SpatialPoint<Scalar, dimention_rank>>();
			}
			begin_point->centor = point;
			begin_point->centor_eular_angles = eularangles;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g) {
			if(!begin_point) {
				begin_point = std::make_unique<Footprint::SpatialPoint<Scalar, dimention_rank>>();
			}
			begin_point->centor << x, y, z;
			begin_point->centor_eular_angles << a, b, g;
			return *this;
		}

		template class FootprintPlannerBase<float>;
		template class FootprintPlannerBase<double>;
	}
}

