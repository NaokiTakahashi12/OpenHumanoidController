
/**
  *
  * @file HumanoidFootprintManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "HumanoidFootprintManager.hpp"

#include <stdexcept>

namespace FootStepPlanner {
	template <typename Scalar>
	HumanoidFootprintManager<Scalar>::~HumanoidFootprintManager() {
	}

	template <typename Scalar>
	typename HumanoidFootprintManager<Scalar>::Ptr HumanoidFootprintManager<Scalar>::make_ptr() {
		return std::make_unique<HumanoidFootprintManager>();
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::register_footprint_planner(PlannerPtr new_footprint_planner) {
		footprint_planner = std::move(new_footprint_planner);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::generate_footprint() {
		if(!footprint_planner) {
			throw std::runtime_error("Can not regist footprint planner from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->full_step();
	}

	template <typename Scalar>
	typename HumanoidFootprintManager<Scalar>::Planner::FootprintList HumanoidFootprintManager<Scalar>::get_footprint_list() {
		if(!footprint_planner) {
			throw std::runtime_error("Can not regist footprint planner from FootStepPlanner::HumanoidFootprintManager");
		}
		return footprint_planner->get_footprint_list();
	}

	template class HumanoidFootprintManager<float>;
	template class HumanoidFootprintManager<double>;
}

