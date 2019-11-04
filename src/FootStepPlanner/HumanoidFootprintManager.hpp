
/**
  *
  * @file HumanoidFootprintManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <Tools/Math/Matrix.hpp>

#include "FootprintPlanner/Humanoid.hpp"
#include "FootprintPlanner/PlanSelector.hpp"

namespace FootStepPlanner {
	template <typename Scalar>
	class HumanoidFootprintManager {
		private :
			using Selector = FootprintPlanner::PlanSelector<Scalar>;
			using SelectorPtr = typename Selector::Ptr;

			using Planner = typename FootprintPlanner::PlanSelector<Scalar>::HoldType;

		public :
			using PlannerPtr = typename Planner::Ptr;

			using Ptr = std::unique_ptr<HumanoidFootprintManager>;
			
			HumanoidFootprintManager();
			virtual ~HumanoidFootprintManager();

			static Ptr make_ptr();

			void register_footprint_planner(const typename Selector::Key &, PlannerPtr);
			bool is_registered_footprint_planner(const typename Selector::Key &);

			void choice_footprint_planner(const std::string &config_file_name);

			void set_goal(const typename Planner::Vector &, const typename Planner::EulerAngles &);
			void set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &roll, const Scalar &pitch, const Scalar &yaw);
			void set_begin(const typename Planner::Vector &, const typename Planner::EulerAngles &);
			void set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &roll, const Scalar &pitch, const Scalar &yaw);

			void make_full_footprint();

			typename Planner::FootprintList get_footprint_list();

		private :
			PlannerPtr footprint_planner;
			SelectorPtr footprint_plan_selector;

			//! Debug
			typename Planner::Vector goal_point, begin_point;
			typename Planner::EulerAngles goal_eulerangle, begin_eulerangle;

			void register_default_footprint_planners();

	};
}

