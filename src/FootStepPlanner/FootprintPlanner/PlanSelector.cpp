
/**
  *
  * @file PlanSelector.cpp
  * @author Naoki Takahashi
  *
  **/

#include "PlanSelector.hpp"

#include <stdexcept>
#include <iostream>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename T>
		PlanSelector<T>::PlanSelector() {
		}

		template <typename T>
		PlanSelector<T>::~PlanSelector() {
		}

		template <typename T>
		typename PlanSelector<T>::Ptr PlanSelector<T>::make_ptr() {
			return std::make_unique<PlanSelector<T>>();
		}

		template <typename T>
		void PlanSelector<T>::register_planner(const Key &key, HoldPtr pointer) {
			if(!pointer) {
				throw std::runtime_error("Failed register planner from FootStepPlanner::FootprintPlanner::PlanSelector");
			}
			pointer_map[key] = std::move(pointer);
		}

		template <typename T>
		bool PlanSelector<T>::is_registered(const Key &key) {
			return pointer_map.find(key) != pointer_map.cend();
		}

		template <typename T>
		typename PlanSelector<T>::HoldPtr PlanSelector<T>::get_planner(const Key &key) {
			return std::move(pointer_map[key]);
		}

		template class PlanSelector<float>;
		template class PlanSelector<double>;
	}
}

