
/**
  *
  * @file LinerInvertedPendulum.cpp
  * @author Naoki Takahashi
  *
  **/

#include "LinerInvertedPendulum.hpp"

#include <iostream>
#include <limits>
#include <chrono>

namespace TrajectoryPattern {
	LinerInvertedPendulum::LinerInvertedPendulum() {
	}

	void LinerInvertedPendulum::compute(const MatrixElement &one_leg_holding_time, const MatrixElement &interval_time, const MatrixElement &a, const MatrixElement &b) {
		this->one_leg_holding_time = one_leg_holding_time,
		this->interval_time = interval_time,
		this->weight_a = a,
		this->weight_b = b;

		time_constant = sqrt(com_z_hight/gravity);

		com_position = Vector2::Constant(std::numeric_limits<MatrixElement>::min());
		before_position = com_position;

		com_velocity = com_velocity.Zero();
		before_velocity = com_velocity;

		footprint = footprint.Zero();
		modiy_footprint = footprint;

		com_line.clear();
		changed_footprint.clear();

		for(int i = 0; i < static_cast<int>(footprint_list.size() - 1); i ++) {
			static std::chrono::high_resolution_clock::time_point start_point, end_point;
			start_point = std::chrono::high_resolution_clock::now();
			iterate_footprint(i);
			end_point = std::chrono::high_resolution_clock::now();
			//this->interval_time = std::chrono::duration<MatrixElement, std::milli>(end_point - start_point).count();
		}
	}

	void LinerInvertedPendulum::set_com_hight(const MatrixElement &com_z_hight) {
		this->com_z_hight = com_z_hight;
	}

	void LinerInvertedPendulum::set_footprint_list(FootPrintList &new_footprint_list) {
		for(auto &fsl : new_footprint_list) {
			this->footprint_list.push_back(fsl);
		}
	}

	void LinerInvertedPendulum::iterate_footprint(const int &number_of_footprint) {
		static Tools::Math::Matrix<MatrixElement, 2, 3> constant_position_matrix23;
		static Tools::Math::Matrix<MatrixElement, 2, 2> constant_velocity_matrix22;
		static Vector2 diffrent_of_foot_position;
		{
			diffrent_of_foot_position = before_position - modiy_footprint;
			constant_position_matrix23 << diffrent_of_foot_position, time_constant * before_velocity, modiy_footprint;
			constant_velocity_matrix22 << diffrent_of_foot_position / time_constant, before_velocity;
		}
		for(MatrixElement current_time = 0; current_time <= one_leg_holding_time; current_time += interval_time) {
			//static std::chrono::high_resolution_clock::time_point start_point, end_point;
			//start_point = std::chrono::high_resolution_clock::now();
			static Tools::Math::Matrix<MatrixElement, 3, 1> constant_hypebolic_vector3;
			static Tools::Math::Matrix<MatrixElement, 2, 1> constant_hypebolic_vector2;
			const auto time_time_constant = current_time / time_constant;
			const auto cosh_t = std::cosh(time_time_constant),
				  	   sinh_t = std::sinh(time_time_constant);

			constant_hypebolic_vector3 << cosh_t, sinh_t, 1;
			constant_hypebolic_vector2 << sinh_t, cosh_t;
			com_position = constant_position_matrix23 * constant_hypebolic_vector3;
			com_velocity = constant_velocity_matrix22 * constant_hypebolic_vector2;
			com_line.push_back(Vector2(com_position));
			//end_point = std::chrono::high_resolution_clock::now();
			//current_time += std::chrono::duration<MatrixElement, std::ratio<1, 1>>(end_point - start_point).count();
		}
		before_position = com_position;
		before_velocity = com_velocity;

		//Foot landing point
		footprint = footprint_list.at(number_of_footprint);

		//Walk fragment
		distance_of_footprint = 0.5 * (footprint_list.at(number_of_footprint + 1) - footprint_list.at(number_of_footprint));
		const auto cosh_one_leg_holding_time = std::cosh(one_leg_holding_time / time_constant),
				   sinh_one_leg_holding_time = std::sinh(one_leg_holding_time / time_constant);
		static Tools::Math::Matrix<MatrixElement, 2, 2> walk_fragment_velocity_matrix22;
		static Tools::Math::Matrix<MatrixElement, 2, 1> walk_fragment_footprint_position;

		walk_fragment_velocity_matrix22 << cosh_one_leg_holding_time + 1, 0, 0, cosh_one_leg_holding_time - 1;
		walk_fragment_footprint_position = 1 / (time_constant * sinh_one_leg_holding_time) * distance_of_footprint;
		walk_fragment_velocity = walk_fragment_velocity_matrix22 * walk_fragment_footprint_position;

		//State of goal
		static Vector2 xdyd;
		xdyd = footprint + distance_of_footprint;

		//Modifycate foot print
		const auto large_d = weight_a * pow((cosh_one_leg_holding_time - 1), 2) + weight_b * pow((sinh_one_leg_holding_time / time_constant), 2);
		modiy_footprint << 
		- weight_a * (cosh_one_leg_holding_time - 1) / large_d
		* (xdyd.x() - cosh_one_leg_holding_time * before_position.x() - time_constant * sinh_one_leg_holding_time * before_velocity.x())
		- weight_b * sinh_one_leg_holding_time / (time_constant * large_d)
		* (walk_fragment_velocity.x() - sinh_one_leg_holding_time / time_constant * before_position.x() - cosh_one_leg_holding_time * before_velocity.x()),

		- weight_a * (cosh_one_leg_holding_time - 1) / large_d
		* (xdyd.y() - cosh_one_leg_holding_time * before_position.y() - time_constant * sinh_one_leg_holding_time * before_velocity.y())
		- weight_b * sinh_one_leg_holding_time / (time_constant * large_d)
		* (walk_fragment_velocity.y() - sinh_one_leg_holding_time / time_constant * before_position.y() - cosh_one_leg_holding_time * before_velocity.y());

		changed_footprint.push_back(Vector2(modiy_footprint));
	}
}


