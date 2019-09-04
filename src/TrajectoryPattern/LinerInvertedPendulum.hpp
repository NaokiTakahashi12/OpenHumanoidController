
/**
  *
  * @file LinerInvertedPendulum.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <cmath>
#include <vector>
#include <deque>

#include <Tools/Math/Matrix.hpp>

namespace TrajectoryPattern {
	class LinerInvertedPendulum {
		public :
			using MatrixElement = long double;
			using Vector2 = Tools::Math::Vector2<MatrixElement>;
			using FootPrintList = std::vector<Vector2>;

			LinerInvertedPendulum();

			void compute(
					const MatrixElement &one_leg_holding_time,
					const MatrixElement &interval_time,
					const MatrixElement &weight_a,
					const MatrixElement &weight_b
			);

			void set_com_hight(const MatrixElement &);

			void set_footprint_list(FootPrintList &);

			std::vector<Vector2> com_line,
								 changed_footprint;

		private :
			static constexpr MatrixElement gravity = 9.80665;

			MatrixElement com_z_hight,
				  		  one_leg_holding_time,
						  interval_time,
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
					modiy_footprint;

			FootPrintList footprint_list;

			void iterate_footprint(const int &);
			 
	};
}

