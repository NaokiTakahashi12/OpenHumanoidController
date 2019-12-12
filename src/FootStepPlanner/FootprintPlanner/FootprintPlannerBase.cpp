
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
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_goal(const decltype((goal_point)) spatial_point) {
			if(!spatial_point) {
				throw std::runtime_error("Can not access spatial_point from FootStepPlanner::FootprintPlanner::FootprintPlannerBase");
			}
			if(!goal_point) {
				goal_point = SpatialPoint::make_ptr();
			}
			goal_point->centor = spatial_point->centor;
			goal_point->centor_eular_angles = spatial_point->centor_eular_angles;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_goal(const Vector &point, const EulerAngles &eularangles) {
			if(!goal_point) {
				goal_point = SpatialPoint::make_ptr();
			}
			goal_point->centor = point;
			goal_point->centor_eular_angles = eularangles;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g) {
			if(!goal_point) {
				goal_point = SpatialPoint::make_ptr();
			}
			goal_point->centor << x, y, z;
			goal_point->centor_eular_angles << a, b, g;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_begin(const decltype((begin_point)) spatial_point) {
			if(!spatial_point) {
				throw std::runtime_error("Can not access spatial_point from FootStepPlanner::FootStepPlanner::FootprintPlannerBase");
			}
			if(!begin_point) {
				begin_point = SpatialPoint::make_ptr();
			}
			begin_point->centor = spatial_point->centor;
			begin_point->centor_eular_angles = spatial_point->centor_eular_angles;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_begin(const Vector &point, const EulerAngles &eularangles) {
			if(!begin_point) {
				begin_point = SpatialPoint::make_ptr();
			}
			begin_point->centor = point;
			begin_point->centor_eular_angles = eularangles;

			return *this;
		}

		template <typename Scalar>
		FootprintPlannerBase<Scalar> &FootprintPlannerBase<Scalar>::set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &a, const Scalar &b, const Scalar &g) {
			if(!begin_point) {
				begin_point = SpatialPoint::make_ptr();
			}
			begin_point->centor << x, y, z;
			begin_point->centor_eular_angles << a, b, g;

			return *this;
		}

		template <typename Scalar>
		void FootprintPlannerBase<Scalar>::config(const std::string &new_config_file_name) {
			config_manager = ConfigManager::make_ptr(new_config_file_name);
		}

		template <typename Scalar>
		void FootprintPlannerBase<Scalar>::clear_footprint() {
			throw std::runtime_error("Unoverride from FootStepPlanner::FootprintPlanner::FootprintPlannerBase");
		}

		template class FootprintPlannerBase<float>;
		template class FootprintPlannerBase<double>;
	}
}

