
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
#include <tuple>
#include <limits>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		class ConstantRangeBasedHumanoid : public Humanoid<Scalar> {
			public :
				//! Range of [0 - 1]
				using DampingFunction = std::function<Scalar(const Scalar &x)>;

				using Ptr = std::unique_ptr<ConstantRangeBasedHumanoid>;

				using typename Humanoid<Scalar>::CountOfFootstep;

				ConstantRangeBasedHumanoid();
				~ConstantRangeBasedHumanoid();

				static std::string get_key();
				static Ptr make_ptr();

				void footstep_range(const Scalar &);
				const Scalar &footstep_range();

				void register_damping(DampingFunction);

				void maximum_footprint(const CountOfFootstep &);
				const CountOfFootstep &maximum_footprint();

				void upper_limit_of_forward(const Scalar &);
				const Scalar &upper_limit_of_forward();

				void lower_limit_of_forward(const Scalar &);
				const Scalar &lower_limit_of_forward();

				void begin_footstep_interval(const Scalar &interval);

				void config(const std::string &new_config_file_name) override final;

				//! @todo Rotation pose footprint
				void full_step() override final;

			private :
				using Humanoid<Scalar>::dimention_rank;
				using typename Humanoid<Scalar>::Vector;

				using SwitchLandingFoot = bool;
				using ForwardAngles = std::tuple<Scalar, Scalar>;
				using ForwardQuaternions = std::tuple<Eigen::Quaternion<Scalar>, Eigen::Quaternion<Scalar>>;

				static constexpr auto constant_limited_forward_angle = M_PI / 2 - std::numeric_limits<Scalar>::min();

				Scalar normalized_upper_limit_of_forward,
					   normalized_lower_limit_of_forward;

				CountOfFootstep maximum_count_of_footprint,
								current_footstep;

				Vector maximum_footprint_range;

				DampingFunction damping_function;

				SwitchLandingFoot initialize_flag_of_landing();

				Vector distance_of_goal();

				bool is_maximum_step() const;
				bool is_normalized_goal(const Scalar &) const;

				decltype(auto) round_range(const Scalar &x, const Scalar &upper, const Scalar &lower) const noexcept;
				bool round_range(Scalar &x, const Scalar &upper, const Scalar &lower) const noexcept;

				const Scalar &limited_forward_y_angle(const Scalar &normalized_distance) const;
				const Scalar &limited_forward_x_angle(const Scalar &normalized_distance) const;
				void modificate_limited_forward_angles(Scalar &forward_x_angle, Scalar &forward_y_angle, const SwitchLandingFoot &) const;
				void modificate_limited_forward_angles(ForwardAngles &, const SwitchLandingFoot &) const;

				ForwardAngles &generate_limited_forward_angles(const Scalar &normalized_distance, const SwitchLandingFoot &) const;
				ForwardQuaternions &generate_forward_quaternion(const Scalar &normalized_distance, const SwitchLandingFoot &) const;

				Scalar &normalized_distance_to_goal_scalar();
				Vector &normalized_distance_to_goal_vector();
		};
	}
}

