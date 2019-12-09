
/**
  *
  * @file WalkFragments.cpp
  * @author Naoki Takahashi
  *
  **/

#include "WalkFragments.hpp"

#include <chrono>

namespace TrajectoryPattern {
	namespace LinerInvertedPendulum {
		WalkFragments::WalkFragments(RobotStatus::InformationPtr &robot_status_information_ptr) {
			robo_info = robot_status_information_ptr;
		}

		void WalkFragments::compute(const MatrixElement &one_leg_holding_time, const MatrixElement &a, const MatrixElement &b) {
			compute(footprint_size(), one_leg_holding_time, a, b);
		}

		void WalkFragments::compute(const unsigned int &number_of_footprint, const MatrixElement &one_leg_holding_time, const MatrixElement &a, const MatrixElement &b) {
			this->one_leg_holding_time = one_leg_holding_time,
			this->weight_a = a,
			this->weight_b = b;

			initialize();

			if(number_of_footprint > footprint_size()) {
				throw std::invalid_argument("Failed argument size(" + std::to_string(number_of_footprint) + ") from TrajectoryPattern::LinerInvertedPendulum::WalkFragments");
			}

			for(unsigned int i = 0; i < number_of_footprint - 1; i ++) {
				iterate_footprint(i);
			}

			update_finish_foot_trajectory();
		}

		void WalkFragments::set_raising_foot(const MatrixElement &raising_foot) {
			this->raising_foot = raising_foot;
		}

		void WalkFragments::set_delay_raising_foot(const MatrixElement &delay_raising_foot) {
			this->delay_raising_foot = delay_raising_foot;
		}

		void WalkFragments::set_com_hight(const MatrixElement &com_z_hight) {
			this->com_z_hight = com_z_hight;
		}

		void WalkFragments::set_footprint_list(const Footprints &left, const Footprints &right, const bool &priority_left) {
			if(left.size() != right.size()) {
				throw std::invalid_argument("Failed differet size of footprint from TrajectoryPattern::LinerInvertedPendulum::WalkFragments");
			}
			left_footprint = left_footprint.Zero(3, left.cols());
			right_footprint = right_footprint.Zero(3, right.cols());

			left_footprint = left;
			right_footprint = right;
			this->priority_left = priority_left;
		}

		void WalkFragments::iterate_footprint(const unsigned int &number_of_footprint) {
			footprint = get_footprint(number_of_footprint);
			modified_footprint = footprint;
			footprint_modificator(number_of_footprint);

			static std::chrono::steady_clock::time_point start_point, end_point;
			for(MatrixElement current_time = 0; current_time < one_leg_holding_time;) {
				start_point = std::chrono::steady_clock::now();

				update_traject_com(current_time);
				update_swing_foot_trajectory(current_time, number_of_footprint);

				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				end_point = std::chrono::steady_clock::now();
				current_time += std::chrono::duration<MatrixElement, std::ratio<1, 1>>(end_point - start_point).count();
			}

			before_position = com_position;
			before_velocity = com_velocity;
			before_modified_footprint = modified_footprint;
		}

		void WalkFragments::update_traject_com(const MatrixElement &current_time) {
			static Tools::Math::Matrix<MatrixElement, 2, 3> constant_position_matrix23;
			static Tools::Math::Matrix<MatrixElement, 2, 2> constant_velocity_matrix22;
			static Tools::Math::Matrix<MatrixElement, 3, 1> constant_hypebolic_vector3;
			static Tools::Math::Matrix<MatrixElement, 2, 1> constant_hypebolic_vector2;

			const auto constant_t_time = current_time / time_constant;
			const auto cosh_t = std::cosh(constant_t_time),
					   sinh_t = std::sinh(constant_t_time);
			const auto diffrent_of_foot_position = before_position - modified_footprint;

			constant_position_matrix23 << diffrent_of_foot_position, time_constant * before_velocity, modified_footprint;
			constant_velocity_matrix22 << diffrent_of_foot_position / time_constant, before_velocity;
			constant_hypebolic_vector3 << cosh_t, sinh_t, 1;
			constant_hypebolic_vector2 << sinh_t, cosh_t;

			com_position = constant_position_matrix23 * constant_hypebolic_vector3;
			com_velocity = constant_velocity_matrix22 * constant_hypebolic_vector2;

			set_com_trajectory_point();
		}

		void WalkFragments::footprint_modificator(const unsigned int &number_of_footprint) {
			static Vector2 xdyd;
			static Tools::Math::Matrix<MatrixElement, 2, 2> walk_fragment_velocity_matrix22;
			static Tools::Math::Matrix<MatrixElement, 2, 1> walk_fragment_footprint_position;

			const auto cosh_one_leg_holding_time = std::cosh(one_leg_holding_time / time_constant),
					   sinh_one_leg_holding_time = std::sinh(one_leg_holding_time / time_constant);

			distance_of_footprint = 0.5 * (get_footprint(number_of_footprint + 1) - get_footprint(number_of_footprint));

			walk_fragment_velocity_matrix22 << 
				cosh_one_leg_holding_time + 1, 0,
				0, cosh_one_leg_holding_time - 1;

			walk_fragment_footprint_position = 1 / (time_constant * sinh_one_leg_holding_time) * distance_of_footprint;
			walk_fragment_velocity = walk_fragment_velocity_matrix22 * walk_fragment_footprint_position;

			xdyd = footprint + distance_of_footprint;

			const auto large_d =
				weight_a * pow((cosh_one_leg_holding_time - 1), 2)
				+ weight_b * pow((sinh_one_leg_holding_time / time_constant), 2);

			modified_footprint << 
			- weight_a * (cosh_one_leg_holding_time - 1) / large_d
			* (xdyd.x() - cosh_one_leg_holding_time * before_position.x() - time_constant * sinh_one_leg_holding_time * before_velocity.x())
			- weight_b * sinh_one_leg_holding_time / (time_constant * large_d)
			* (walk_fragment_velocity.x() - sinh_one_leg_holding_time / time_constant * before_position.x() - cosh_one_leg_holding_time * before_velocity.x()),
			- weight_a * (cosh_one_leg_holding_time - 1) / large_d
			* (xdyd.y() - cosh_one_leg_holding_time * before_position.y() - time_constant * sinh_one_leg_holding_time * before_velocity.y())
			- weight_b * sinh_one_leg_holding_time / (time_constant * large_d)
			* (walk_fragment_velocity.y() - sinh_one_leg_holding_time / time_constant * before_position.y() - cosh_one_leg_holding_time * before_velocity.y());

			set_modified_footprint(number_of_footprint, modified_footprint);
		}

		void WalkFragments::initialize() {
			time_constant = sqrt(com_z_hight/gravity);

			com_position = get_footprint(0);
			before_position = com_position;

			com_velocity = com_velocity.Constant(1) * 1e-9;
			before_velocity = com_velocity;

			com_line.clear();

			modified_left_footprint = modified_left_footprint.Zero(3, left_footprint.cols());
			modified_right_footprint = modified_right_footprint.Zero(3, left_footprint.cols());
		}

		void WalkFragments::update_swing_foot_trajectory(const MatrixElement &current_time, const unsigned int &number_of_footprint) {
			Tools::Math::Vector3<MatrixElement> left_foot, right_foot;
			left_foot = Tools::Math::Vector3<MatrixElement>::Zero();
			right_foot = Tools::Math::Vector3<MatrixElement>::Zero();
			auto next_landing_point = get_footprint(number_of_footprint + 1);

			if(number_of_footprint % 2 ^ priority_left) {
				left_foot.x() = modified_footprint.x() - com_position.x();
				left_foot.y() = modified_footprint.y() - com_position.y();

				right_foot.x() = next_landing_point.x() - com_position.x();
				right_foot.y() = next_landing_point.y() - com_position.y();

				if(current_time >= delay_raising_foot) {
					right_foot.z() = raising_foot;
				}
			}
			else {
				right_foot.x() = modified_footprint.x() - com_position.x();
				right_foot.y() = modified_footprint.y() - com_position.y();

				left_foot.x() = next_landing_point.x() - com_position.x();
				left_foot.y() = next_landing_point.y() - com_position.y();

				if(current_time >= delay_raising_foot) {
					left_foot.z() = raising_foot;
				}
			}

			robo_info->left_foot_trajectory->set(left_foot);
			robo_info->right_foot_trajectory->set(right_foot);
		}

		void WalkFragments::update_finish_foot_trajectory() {
			Tools::Math::Vector3<MatrixElement> left_foot, right_foot;

			left_foot = robo_info->left_foot_trajectory->latest().value;
			right_foot = robo_info->right_foot_trajectory->latest().value;
			left_foot.z() = 0;
			right_foot.z() = 0;

			robo_info->left_foot_trajectory->set(left_foot);
			robo_info->right_foot_trajectory->set(right_foot);
		}

		WalkFragments::Vector2 WalkFragments::get_footprint(const unsigned int &number_of_footprint) {
			constexpr auto footprint_dimention = 2;
			const auto access_footprint_number = static_cast<int>(number_of_footprint * 0.5);

			if(number_of_footprint % 2 ^ priority_left) {
				return left_footprint.block<footprint_dimention, 1>(0, access_footprint_number);
			}

			return right_footprint.block<footprint_dimention, 1>(0, access_footprint_number);
		}

		void WalkFragments::set_modified_footprint(const unsigned int &number_of_footprint, const Vector2 &new_modified_footprint) {
			constexpr auto footprint_dimention = 2;
			const auto access_footprint_number = static_cast<int>(number_of_footprint * 0.5);

			if(number_of_footprint % 2 ^ priority_left) {
				modified_left_footprint.block<footprint_dimention, 1>(0, access_footprint_number) = new_modified_footprint;
				modified_left_footprint(2, access_footprint_number) = left_footprint(2, access_footprint_number);
				set_left_modified_footprint();
			}
			else {
				modified_right_footprint.block<footprint_dimention, 1>(0, access_footprint_number) = new_modified_footprint;
				modified_right_footprint(2, access_footprint_number) = right_footprint(2, access_footprint_number);
				set_right_modified_footprint();
			}
		}

		void WalkFragments::set_com_trajectory_point() {
			static Tools::Math::Vector3<float> com_trajectory_point;

			com_line.push_back(Vector2(com_position));

			if(robo_info->com_trajectory) {
				com_trajectory_point.x() = com_position.x();
				com_trajectory_point.y() = com_position.y();
				com_trajectory_point.z() = com_z_hight;
				robo_info->com_trajectory->set(com_trajectory_point);
			}
		}

		void WalkFragments::set_left_modified_footprint() {
			if(robo_info->left_modified_footprint) {
				robo_info->left_modified_footprint->set(modified_left_footprint);
			}
		}

		void WalkFragments::set_right_modified_footprint() {
			if(robo_info->right_modified_footprint) {
				robo_info->right_modified_footprint->set(modified_right_footprint);
			}
		}

		unsigned int WalkFragments::footprint_size() {
			return left_footprint.cols() + right_footprint.cols();
		}
	}
}


