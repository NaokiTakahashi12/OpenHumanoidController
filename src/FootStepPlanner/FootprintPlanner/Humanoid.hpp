
/**
  *
  * @file Humanoid.hpp
  * @brief Humanoid footprint base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "FootprintPlannerBase.hpp"

#include <array>
#include <deque>

#include "../Footprint/Bipedal.hpp"

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		class Humanoid : public FootprintPlannerBase<Scalar> {
			protected :
				using FootprintPlannerBase<Scalar>::dimention_rank;
				using typename FootprintPlannerBase<Scalar>::Vector;
				using typename FootprintPlannerBase<Scalar>::EulerAngles;

				using CountOfFootstep = unsigned int;

			public :
				using OnceFootprint = Footprint::Bipedal<Scalar>;
				using FootprintList = std::deque<OnceFootprint>;
				using Ptr = std::unique_ptr<Humanoid>;

				Humanoid();
				virtual ~Humanoid();

				FootprintList get_footprint_list() const;

				//! @todo Return status
				virtual void full_step() = 0;

			protected :
				OnceFootprint current_bipedal_point;
				FootprintList footprint_list;

				void initialize_body_point(const Scalar &maximum_distance_of_foot, const Scalar &interval = 2);

				void register_current_frame();
		};
	}
}

