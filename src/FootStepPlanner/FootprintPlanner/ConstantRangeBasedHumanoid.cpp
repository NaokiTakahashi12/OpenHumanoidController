
/**
  *
  * @file  ConstantRangeBasedHumanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ConstantRangeBasedHumanoid.hpp"

#include <cmath>
#include <limits>


namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		ConstantRangeBasedHumanoid<Scalar>::ConstantRangeBasedHumanoid() {
			normalized_upper_limit_of_forward = 1;
			maximum_count_of_footprint = 30;
		}

		template <typename Scalar>
		ConstantRangeBasedHumanoid<Scalar>::~ConstantRangeBasedHumanoid() {
		}

		template <typename Scalar>
		std::string ConstantRangeBasedHumanoid<Scalar>::get_key() {
			return "ConstantRangeBasedHumanoid";
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::Ptr ConstantRangeBasedHumanoid<Scalar>::make_ptr() {
			return std::make_unique<ConstantRangeBasedHumanoid>();
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::footstep_range(const Scalar &new_maximum_footprint_range) {
			maximum_footprint_range.x() = new_maximum_footprint_range;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::footstep_range() {
			return maximum_footprint_range.x();
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::register_damping(DampingFunction new_damping_function) {
			damping_function = new_damping_function;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::maximum_footprint(const CountOfFootstep &new_maximum_count_of_footprint) {
			maximum_count_of_footprint = new_maximum_count_of_footprint;
		}

		template <typename Scalar>
		const typename ConstantRangeBasedHumanoid<Scalar>::CountOfFootstep &ConstantRangeBasedHumanoid<Scalar>::maximum_footprint() {
			return maximum_count_of_footprint;
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
			if(maximum_footprint_range.x() == 0) {
				throw std::runtime_error("Footstep range is zero from FootStepPlanner::FootprintPlanner::ConstantRangeBasedHumanoid");
			}
			this->initialize_body_point(maximum_footprint_range.x(), interval);
			this->register_current_frame();
			current_footstep = 1;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::config(const std::string &new_config_file_name) {
			if(this->config_manager) {
				this->config_manager.reset();
			}
			this->config_manager = ConfigManager::make_ptr(new_config_file_name);

			if(!this->config_manager) {
				throw std::runtime_error("Can not access to config_manager from FootStepPlanner::FootprintPlanner::ConstantRangeBasedHumanoid");
			}
			const std::string tree_top_identity = "Humanoid.",
				              this_tree_identity = "ConstantRangeBasedHumanoid.";
			footstep_range(
				this->config_manager->template get_value<Scalar>(
					tree_top_identity + this_tree_identity + "Foot step range"
				)
			);
			maximum_footprint(
				this->config_manager->template get_value<int>(
					tree_top_identity + this_tree_identity + "Maximum number of footprint"
				)
			);
			upper_limit_of_forward(
				this->config_manager->template get_value<Scalar>(
					tree_top_identity + this_tree_identity + "Forward upper limit"
				)
			);
			lower_limit_of_forward(
				this->config_manager->template get_value<Scalar>(
					tree_top_identity + this_tree_identity + "Forward lower limit"
				)
			);
			const std::string forward_function_type = this->config_manager->template get_value<std::string>(
					tree_top_identity + this_tree_identity + "Forward function type"
			);

			if("sin curve" == forward_function_type) {
				register_damping(
					[](const double &x) {
						return std::abs(std::sin(M_PI * x));
					}
				);
			}
			else if("cos curve" == forward_function_type) {
				register_damping(
					[](const double &x) {
						return std::abs(std::cos(M_PI * x));
					}
				);
			}
			else {
				throw std::runtime_error("Unknown type curve from FootStepPlanner::FootprintPlanner::ConstantRangeBasedHumanoid");
			}
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::full_step() {
			if(maximum_footprint_range.x() == 0) {
				throw std::runtime_error("Footstep range is zero from FootStepPlanner::ConstantRangeBasedHumanoid");
			}
			if(!damping_function) {
				throw std::runtime_error("Can not call function from FootStepPlanner::ConstantRangeBasedHumanoid");
			}
			current_footstep = 0;
			this->clear_footprint();

			//! @todo initialize point
			begin_footstep_interval(2);

			static Eigen::Quaternion<Scalar> forward_x_q, forward_y_q;
			auto landing_switch_flag = initialize_flag_of_landing();
			
			while(1) {
				static Eigen::Quaternion<Scalar> forward_q;
				const auto current_distance = normalized_distance_to_goal_scalar();

				if(is_maximum_step() || is_normalized_goal(current_distance)) {
					break;
				}

				std::tie(forward_x_q, forward_y_q) = generate_forward_quaternion(current_distance, landing_switch_flag);

				if(!std::exchange(landing_switch_flag, true)) {
					forward_q = forward_x_q * forward_y_q;
					this->current_bipedal_point.left = forward_q * maximum_footprint_range + this->current_bipedal_point.right;
					this->current_bipedal_point.centor = forward_q * maximum_footprint_range / 2 + this->current_bipedal_point.right;
				}
				else if(std::exchange(landing_switch_flag, false)) {
					forward_q = forward_x_q.inverse() * forward_y_q;
					this->current_bipedal_point.right = forward_q * maximum_footprint_range + this->current_bipedal_point.left;
					this->current_bipedal_point.centor = forward_q * maximum_footprint_range / 2 + this->current_bipedal_point.left;
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
		bool ConstantRangeBasedHumanoid<Scalar>::is_maximum_step() const {
			return current_footstep >= maximum_count_of_footprint;
		}

		template <typename Scalar>
		bool ConstantRangeBasedHumanoid<Scalar>::is_normalized_goal(const Scalar &normalized_distance) const {
			return normalized_distance >= 1;
		}

		template <typename Scalar>
		decltype(auto) ConstantRangeBasedHumanoid<Scalar>::round_range(const Scalar &x, const Scalar &upper, const Scalar &lower) const noexcept {
			static Scalar ret;

			round_range(ret = x, upper, lower);

			return ret;
		}

		template <typename Scalar>
		bool ConstantRangeBasedHumanoid<Scalar>::round_range(Scalar &x, const Scalar &upper, const Scalar &lower) const noexcept {
			if(upper < x) {
				x = upper;
			}
			else if(lower > x) {
				x = lower;
			}
			else {
				return false;
			}
			return true;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::limited_forward_x_angle(const Scalar &normalized_distance) const {
			static Scalar ret;

			ret = constant_limited_forward_angle - constant_limited_forward_angle * damping_function(normalized_distance);

			return ret;
		}

		template <typename Scalar>
		const Scalar &ConstantRangeBasedHumanoid<Scalar>::limited_forward_y_angle(const Scalar &normalized_distance) const {
			static Scalar ret;

			const auto distance = this->goal_point->centor - this->current_bipedal_point.centor;
			const auto yaw_distance = std::asin(distance.y() / distance.template lpNorm<2>());

			ret = yaw_distance * (1 + damping_function(normalized_distance));

			return ret;
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::modificate_limited_forward_angles(Scalar &forward_x_angle, Scalar &forward_y_angle, const SwitchLandingFoot &) const {
			const auto upper_limit = constant_limited_forward_angle * normalized_upper_limit_of_forward;
			const auto lower_limit = constant_limited_forward_angle * normalized_lower_limit_of_forward;

			round_range(forward_x_angle, upper_limit, lower_limit);

			if(std::abs(std::abs(forward_y_angle) - std::abs(forward_x_angle)) <= lower_limit) {
				forward_x_angle += lower_limit - (-std::abs(forward_y_angle) + forward_x_angle);
				round_range(forward_x_angle, upper_limit, lower_limit);
			}
		}

		template <typename Scalar>
		void ConstantRangeBasedHumanoid<Scalar>::modificate_limited_forward_angles(ForwardAngles &limited_forward_angles, const SwitchLandingFoot &landing_switch_flag) const {
			static Scalar forward_x_angle, forward_y_angle;

			std::tie(forward_x_angle, forward_y_angle) = limited_forward_angles;

			modificate_limited_forward_angles(forward_x_angle, forward_y_angle, landing_switch_flag);

			limited_forward_angles = std::make_tuple(forward_x_angle, forward_y_angle);
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::ForwardAngles &ConstantRangeBasedHumanoid<Scalar>::generate_limited_forward_angles(const Scalar &normalized_distance, const SwitchLandingFoot &landing_switch_flag) const {
			static ForwardAngles ret;

			ret = std::make_tuple(limited_forward_x_angle(normalized_distance), limited_forward_y_angle(normalized_distance));
			modificate_limited_forward_angles(ret, landing_switch_flag);

			return ret;
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::ForwardQuaternions &ConstantRangeBasedHumanoid<Scalar>::generate_forward_quaternion(const Scalar &normalized_distance, const SwitchLandingFoot &landing_switch_flag) const {
			static ForwardQuaternions ret;
			const auto &limited_forward_angles = generate_limited_forward_angles(normalized_distance, landing_switch_flag);

			ret = std::make_tuple(
					Eigen::AngleAxis(std::get<0>(limited_forward_angles), maximum_footprint_range.UnitZ()),
					Eigen::AngleAxis(std::get<1>(limited_forward_angles), maximum_footprint_range.UnitZ())
			);

			return ret;
		}

		template <typename Scalar>
		Scalar &ConstantRangeBasedHumanoid<Scalar>::normalized_distance_to_goal_scalar() {
			const auto distance = distance_of_goal();
			static Scalar ret;

			ret = (this->current_bipedal_point.centor - this->begin_point->centor).template lpNorm<2>() / distance.template lpNorm<2>();

			return ret;
		}

		template <typename Scalar>
		typename ConstantRangeBasedHumanoid<Scalar>::Vector &ConstantRangeBasedHumanoid<Scalar>::normalized_distance_to_goal_vector() {
			static Vector distance;
			static Vector ret;

			distance = distance_of_goal();
			ret = this->current_bipedal_point.centor;

			if(ret.x() != 0 && distance.x() != 0) {
				ret.x() /= distance.x();
			}
			if(ret.y() != 0 && distance.y() != 0) {
				ret.y() /= distance.y();
			}
			if(ret.z() != 0 && distance.z() != 0) {
				ret.z() /= distance.z();
			}

			return ret;
		}

		template class ConstantRangeBasedHumanoid<float>;
		template class ConstantRangeBasedHumanoid<double>;
	}
}

