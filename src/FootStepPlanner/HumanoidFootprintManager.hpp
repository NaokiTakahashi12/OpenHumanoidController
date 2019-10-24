
/**
  *
  * @file HumanoidFootprintManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <Tools/Math/Matrix.hpp>

#include "FootprintPlanner/Humanoid.hpp"

namespace FootStepPlanner {
	template <typename Scalar>
	class HumanoidFootprintManager {
		public :
			using Planner = FootprintPlanner::Humanoid<Scalar>;
			using PlannerPtr = typename Planner::Ptr;

			using Ptr = std::unique_ptr<HumanoidFootprintManager>;
			
			virtual ~HumanoidFootprintManager();

			static Ptr make_ptr();

			void register_footprint_planner(PlannerPtr);

			//void set_goal(const typename Planner::Vector &, const typename Planner::EularAngles &);
			void set_goal(const Tools::Math::Vector3<Scalar> &);

			void generate_footprint();

			typename Planner::FootprintList get_footprint_list();

		private :
			PlannerPtr footprint_planner;

	};
}

