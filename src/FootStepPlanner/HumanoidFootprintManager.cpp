
/**
  *
  * @file HumanoidFootprintManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "HumanoidFootprintManager.hpp"

#include <stdexcept>

#include "ConfigManager.hpp"
#include "FootprintPlanner/ConstantRangeBasedHumanoid.hpp"
#include "FootprintPlanner/CustomHumanoid.hpp"

namespace FootStepPlanner {
	template <typename Scalar>
	HumanoidFootprintManager<Scalar>::HumanoidFootprintManager() {
		footprint_plan_selector = Selector::make_ptr();

		if(!footprint_plan_selector) {
			throw std::runtime_error("Failed make ptr for footprint_plan_selector from FootStepPlanner::HumanoidFootprintManager");
		}
		register_default_footprint_planners();
	}

	template <typename Scalar>
	HumanoidFootprintManager<Scalar>::~HumanoidFootprintManager() {
	}

	template <typename Scalar>
	typename HumanoidFootprintManager<Scalar>::Ptr HumanoidFootprintManager<Scalar>::make_ptr() {
		return std::make_unique<HumanoidFootprintManager>();
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::register_footprint_planner(const typename Selector::Key &key, PlannerPtr planner_ptr) {
		if(!footprint_plan_selector) {
			throw std::runtime_error("Can not access to footprint_plan_selector from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_plan_selector->register_planner(
			key,
			std::move(planner_ptr)
		);
	}

	template <typename Scalar>
	bool HumanoidFootprintManager<Scalar>::is_registered_footprint_planner(const typename Selector::Key &key) {
		if(!footprint_plan_selector) {
			throw std::runtime_error("Can not access to footprint_plan_selector from FootStepPlanner::HumanoidFootprintManager");
		}
		return footprint_plan_selector->is_registered(key);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::choice_footprint_planner(const std::string &config_file_name) {
		if(footprint_planner) {
			throw std::runtime_error("Already choice footprint planner from FootStepPlanner::HumanoidFootprintManager");
		}
		if(!footprint_plan_selector) {
			throw std::runtime_error("Can not access to footprint_plan_selector from FootStepPlanner::HumanoidFootprintManager");
		}
		ConfigManager config_manager(config_file_name);
		const auto choice_key = config_manager.get_value<typename Selector::Key>("Humanoid.Use plan");

		if(!is_registered_footprint_planner(choice_key)) {
			throw std::runtime_error("Not registered " + choice_key + " from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner = std::move(footprint_plan_selector->get_planner(choice_key));

		if(!footprint_planner) {
			throw std::runtime_error("Failed move footprint_planner from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_plan_selector.reset();
		footprint_planner->config(config_file_name);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::set_goal(const typename Planner::Vector &point, const typename Planner::EulerAngles &eularangles) {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->set_goal(point, eularangles);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::set_goal(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &roll, const Scalar &pitch, const Scalar &yaw) {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner form FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->set_goal(x, y, z, roll, pitch, yaw);
	}
	
	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::set_begin(const typename Planner::Vector &point, const typename Planner::EulerAngles &eularangles) {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->set_begin(point, eularangles);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::set_begin(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &roll, const Scalar &pitch, const Scalar &yaw) {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner form FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->set_begin(x, y, z, roll, pitch, yaw);
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::make_full_footprint() {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner from FootStepPlanner::HumanoidFootprintManager");
		}
		footprint_planner->full_step();
	}

	template <typename Scalar>
	typename HumanoidFootprintManager<Scalar>::Planner::FootprintList HumanoidFootprintManager<Scalar>::get_footprint_list() {
		if(!footprint_planner) {
			throw std::runtime_error("Can not access to footprint_planner from FootStepPlanner::HumanoidFootprintManager");
		}
		return footprint_planner->get_footprint_list();
	}

	template <typename Scalar>
	void HumanoidFootprintManager<Scalar>::register_default_footprint_planners() {
		footprint_plan_selector->register_planner(
				FootprintPlanner::ConstantRangeBasedHumanoid<Scalar>::get_key(),
				std::move(FootprintPlanner::ConstantRangeBasedHumanoid<Scalar>::make_ptr())
		);
		footprint_plan_selector->register_planner(
				FootprintPlanner::CustomHumanoid<Scalar>::get_key(),
				std::move(FootprintPlanner::CustomHumanoid<Scalar>::make_ptr())
		);
	}

	template class HumanoidFootprintManager<float>;
	template class HumanoidFootprintManager<double>;
}

