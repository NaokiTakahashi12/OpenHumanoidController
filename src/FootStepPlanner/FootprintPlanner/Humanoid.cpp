
/**
  *
  * @file Humanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Humanoid.hpp"

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		Humanoid<Scalar>::Humanoid() {
		}

		template <typename Scalar>
		Humanoid<Scalar>::~Humanoid() {
		}

		template <typename Scalar>
		typename Humanoid<Scalar>::FootprintList Humanoid<Scalar>::get_footprint_list() const {
			return footprint_list;
		}

		template <typename Scalar>
		void Humanoid<Scalar>::clear_footprint() {
			footprint_list.clear();
		}

		template <typename Scalar>
		void Humanoid<Scalar>::register_current_frame() {
			footprint_list.push_back(current_bipedal_point);
		}

		template <typename Scalar>
		void Humanoid<Scalar>::initialize_body_point(const Scalar &maximum_distance_of_foot, const Scalar &interval) {
			const auto half_distance_of_foot = maximum_distance_of_foot / interval;

			current_bipedal_point.centor = this->begin_point->centor;
			current_bipedal_point.left = current_bipedal_point.centor;
			current_bipedal_point.right = current_bipedal_point.centor;

			current_bipedal_point.left.y() += half_distance_of_foot;
			current_bipedal_point.right.y() -= half_distance_of_foot;
		}

		template class Humanoid<float>;
		template class Humanoid<double>;
	}
}

