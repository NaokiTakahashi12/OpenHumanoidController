
/**
  *
  * @file PlanSelector.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <unordered_map>
#include <string>

#include "Humanoid.hpp"

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename T>
		class PlanSelector {
			public :
				using Ptr = std::unique_ptr<PlanSelector>;
				using Key = std::string;

				using HoldType = Humanoid<T>;
				using HoldPtr = typename HoldType::Ptr;

				PlanSelector();
				virtual ~PlanSelector();

				static Ptr make_ptr();

				void register_planner(const Key &, HoldPtr);
				bool is_registered(const Key &);

				HoldPtr get_planner(const Key &);

			private :
				std::unordered_map<Key, HoldPtr> pointer_map;

		};
	}
}

