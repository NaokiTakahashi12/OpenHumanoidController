
/**
  *
  * @file  ConstantRangeBasedHumanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ConstantRangeBasedHumanoid.hpp"

#include <cmath>
#include <limits>
#include <iostream>


namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		ConstantRangeBasedHumanoid<Scalar>::ConstantRangeBasedHumanoid() {
			normalized_upper_limit_of_forward = 1;
			maximum_count_of_footstep = 30;
		}

		template <typename Scalar>
		ConstantRangeBasedHumanoid<Scalar>::~ConstantRangeBasedHumanoid() {
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::Ptr ConstantRangeBasedHumanoid<Scalar>::make_ptr() {
			return std::make_unique<ConstantRangeBasedHumanoid>();
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::footstep_range(const Scalar &new_maximum_footstep_range) {
			maximum_footstep_range.x() = new_maximum_footstep_range;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::footstep_range() {
			return maximum_footstep_range.x();
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::register_damping(DampingFunction new_damping_function) {
			damping_function = new_damping_function;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::maximum_footstep(const CountOfFootstep &new_maximum_count_of_footstep) {
			maximum_count_of_footstep = new_maximum_count_of_footstep;
		}

		template <typename Scalar>
		const typename ConstantRangeBasedHumanoid<Scalar>::CountOfFootstep &ConstantRangeBasedHumanoid<Scalar>::maximum_footstep() {
			return maximum_count_of_footstep;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::upper_limit_of_forward(const Scalar &limit) {
			normalized_upper_limit_of_forward = limit;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::upper_limit_of_forward() {
			return normalized_upper_limit_of_forward;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::lower_limit_of_forward(const Scalar &limit) {
			normalized_lower_limit_of_forward = limit;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::lower_limit_of_forward() {
			return normalized_lower_limit_of_forward;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::begin_footstep_interval(const Scalar &interval) {
			if(maximum_footstep_range.x() == 0) {
				throw std::runtime_error("Footstep range is zero from FootStepPlanner::ConstantRangeBasedHumanoid");
			}
			this->initialize_body_point(maximum_footstep_range.x(), interval);
			this->register_current_frame();
			current_footstep = 1;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::full_step() {
			if(maximum_footstep_range.x() == 0) {
				throw std::runtime_error("Footstep range is zero from FootStepPlanner::ConstantRangeBasedHumanoid");
			}
			if(!damping_function) {
				throw std::runtime_error("Can not call function from FootStepPlanner::ConstantRangeBasedHumanoid");
			}
			static Eigen::Quaternion<Scalar> forward_x_q, forward_y_q;
			auto landing_switch_flag = initialize_flag_of_landing();
			
			while(1) {
				static Eigen::Quaternion<Scalar> forward_q;
				const auto current_distance = normalized_distance_to_goal_scalar();

				if(current_footstep >= maximum_count_of_footstep || current_distance >= 1 - std::numeric_limits<Scalar>::min()) {
					break;
				}
				const auto forward_y_angle = limited_forward_y_angle(current_distance);
				const auto forward_x_angle = limited_forward_x_angle(current_distance, forward_y_angle);

				forward_x_q = Eigen::AngleAxis(forward_x_angle, maximum_footstep_range.UnitZ());
				forward_y_q = Eigen::AngleAxis(forward_y_angle, maximum_footstep_range.UnitZ());

				if(!std::exchange(landing_switch_flag, true)) {
					forward_q = forward_x_q * forward_y_q;
					this->current_bipedal_point.left = forward_q * maximum_footstep_range + this->current_bipedal_point.right;
					this->current_bipedal_point.centor = forward_q * maximum_footstep_range / 2 + this->current_bipedal_point.right;
				}
				else if(std::exchange(landing_switch_flag, false)) {
					forward_q = forward_x_q.inverse() * forward_y_q;
					this->current_bipedal_point.right = forward_q * maximum_footstep_range + this->current_bipedal_point.left;
					this->current_bipedal_point.centor = forward_q * maximum_footstep_range / 2 + this->current_bipedal_point.left;
				}
				this->register_current_frame();

				current_footstep ++;
			}
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::SwitchLandingFoot ConstantRangeBasedHumanoid<Scalar>::initialize_flag_of_landing() {
			return distance_of_goal().y() <= 0 ? false : true;
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::Vector ConstantRangeBasedHumanoid<Scalar>::distance_of_goal() {
			return this->goal_point->centor - this->begin_point->centor;
		}

		template <typename Scalar>
		Scalar ConstantRangeBasedHumanoid<Scalar>::limited_forward_x_angle(const Scalar &normalized_distance, const Scalar &forward_y_angle) {
			constexpr auto limit_angle = M_PI / 2 - std::numeric_limits<Scalar>::min();
			const auto upper_limit = limit_angle * normalized_upper_limit_of_forward;
			const auto lower_limit = limit_angle * normalized_lower_limit_of_forward;

			auto limited_forward_angle = limit_angle - limit_angle * damping_function(normalized_distance);

			if(upper_limit < limited_forward_angle) {
				limited_forward_angle = upper_limit;
			}
			else if(lower_limit > limited_forward_angle) {
				limited_forward_angle = upper_limit;
			}

			if(std::abs(std::abs(forward_y_angle) - std::abs(limited_forward_angle)) <= lower_limit) {
				limited_forward_angle += lower_limit - (-std::abs(forward_y_angle) + limited_forward_angle);
			}

			return limited_forward_angle;
		}

		template <typename Scalar>
		Scalar ConstantRangeBasedHumanoid<Scalar>::limited_forward_y_angle(const Scalar &normalized_distance) {
			const Vector distance = this->goal_point->centor - this->current_bipedal_point.centor;
			const auto yaw_distance = std::atan2(distance.y(), distance.x());

			return yaw_distance * (2 * damping_function(normalized_distance));
		}

		template <typename Scalar>
		Scalar ConstantRangeBasedHumanoid<Scalar>::normalized_distance_to_goal_scalar() {
			const auto distance = distance_of_goal();
			const auto ret = (this->current_bipedal_point.centor - this->begin_point->centor).template lpNorm<2>() / distance.template lpNorm<2>();

			return ret;
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::Vector ConstantRangeBasedHumanoid<Scalar>::normalized_distance_to_goal_vector() {
			const auto distance = distance_of_goal();
			Vector ret = this->current_bipedal_point.centor;
			
			ret.x() /= distance.x() == 0 ? 1 : distance.x();
			ret.y() /= distance.y() == 0 ? 1 : distance.y();
			ret.z() /= distance.z() == 0 ? 1 : distance.z();

			return ret;
		}

		template class ConstantRangeBasedHumanoid<float>;
		template class ConstantRangeBasedHumanoid<double>;
	}
}

