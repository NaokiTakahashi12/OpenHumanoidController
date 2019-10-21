
/**
  *
  * @file  ConstantRangeBasedHumanoid.hpp
  * @brief Constant range based footprint generator
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "Humanoid.hpp"

#include <functional>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		class ConstantRangeBasedHumanoid : public Humanoid<Scalar> {
			private :
				using Humanoid<Scalar>::dimention_rank;
				using typename Humanoid<Scalar>::Vector;
				using typename Humanoid<Scalar>::CountOfFootstep;

				using SwitchLandingFoot = bool;

			public :
				//! Range of [0 - 1]
				using DampingFunction = std::function<Scalar(const Scalar &x)>;

				using Ptr = std::unique_ptr<ConstantRangeBasedHumanoid>;

				ConstantRangeBasedHumanoid();
				~ConstantRangeBasedHumanoid();

				static Ptr make_ptr();

				void footstep_range(const Scalar &);
				const Scalar &footstep_range();

				void register_damping(DampingFunction);

				void maximum_footstep(const CountOfFootstep &);
				const CountOfFootstep &maximum_footstep();

				void upper_limit_of_forward(const Scalar &);
				const Scalar &upper_limit_of_forward();

				void lower_limit_of_forward(const Scalar &);
				const Scalar &lower_limit_of_forward();

				void begin_footstep_interval(const Scalar &interval);

				//! @todo Rotation footprint
				void full_step() override;

			private :
				Scalar normalized_upper_limit_of_forward,
					   normalized_lower_limit_of_forward;

				CountOfFootstep maximum_count_of_footstep,
								current_footstep;

				Tools::Math::Vector<Scalar, dimention_rank> maximum_footstep_range;

				DampingFunction damping_function;

				SwitchLandingFoot initialize_flag_of_landing();

				Vector distance_of_goal();

				Scalar limited_forward_y_angle(const Scalar &normalized_distance);
				Scalar limited_forward_x_angle(const Scalar &normalized_distance, const Scalar &forward_y_angle);

				Scalar normalized_distance_to_goal_scalar();
				Vector normalized_distance_to_goal_vector();
		};
	}
}

