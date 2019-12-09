
/**
  *
  * @file WalkFragments.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <cmath>
#include <vector>
#include <deque>

#include <Tools/Math/Matrix.hpp>

#include <RobotStatus/Information.hpp>

namespace TrajectoryPattern {
	namespace LinerInvertedPendulum {
		class WalkFragments {
			public :
				using MatrixElement = float;
				using Vector2 = Tools::Math::Vector2<MatrixElement>;
				using Footprints = Tools::Math::MatrixX<MatrixElement>;

				WalkFragments(RobotStatus::InformationPtr &);

				void compute(
					const MatrixElement &one_leg_holding_time,
					const MatrixElement &weight_a,
					const MatrixElement &weight_b
				);

				void compute(
					const unsigned int &number_of_footprint,
					const MatrixElement &one_leg_holding_time,
					const MatrixElement &weight_a,
					const MatrixElement &weight_b
				);

				void set_raising_foot(const MatrixElement &);
				void set_delay_raising_foot(const MatrixElement &);

				void set_com_hight(const MatrixElement &);

				void set_footprint_list(const Footprints &left, const Footprints &right, const bool &priority_left = true);

				std::vector<Vector2> com_line;

			private :
				static constexpr MatrixElement gravity = 9.80665;

				RobotStatus::InformationPtr robo_info;

				bool priority_left;

				MatrixElement com_z_hight,
							  raising_foot,
							  delay_raising_foot,
							  one_leg_holding_time,
							  time_constant;

				MatrixElement weight_a,
							  weight_b;

				Vector2 com_position,
						before_position,
						distance_of_footprint,
						com_velocity,
						before_velocity,
						walk_fragment_velocity,
						footprint,
						modified_footprint,
						before_modified_footprint;

				Footprints footprint_matrix,
						   left_footprint,
						   right_footprint,
						   modified_left_footprint,
						   modified_right_footprint;


				void iterate_footprint(const unsigned int &);
				void update_traject_com(const MatrixElement &current_time);
				void footprint_modificator(const unsigned int &number_of_footprint);
				void update_swing_foot_trajectory(const MatrixElement &current_time, const unsigned int &number_of_footprint);
				void update_finish_foot_trajectory();

				void initialize();

				Vector2 get_footprint(const unsigned int &number_of_footprint);
				void set_modified_footprint(const unsigned int &number_of_footprint, const Vector2 &);

				void set_com_trajectory_point();
				void set_left_modified_footprint();
				void set_right_modified_footprint();

				unsigned int footprint_size();

		};
	}
}

