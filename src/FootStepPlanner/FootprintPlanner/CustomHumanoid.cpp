
/**
  *
  * @file CustomHumanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "CustomHumanoid.hpp"

#include <stdexcept>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		CustomHumanoid<Scalar>::CustomHumanoid() {
		}

		template <typename Scalar>
		CustomHumanoid<Scalar>::~CustomHumanoid() {
		}

		template <typename Scalar>
		typename CustomHumanoid<Scalar>::Ptr CustomHumanoid<Scalar>::make_ptr() {
			return std::make_unique<CustomHumanoid>();
		}

		template <typename Scalar>
		std::string CustomHumanoid<Scalar>::get_key() {
			return "CustomHumanoid";
		}
		
		template <typename Scalar>
		void CustomHumanoid<Scalar>::full_step() {
			throw std::runtime_error("Not implemented from FootStepPlanner::FootprintPlanner::CustomHumanoid");
		}

		template class CustomHumanoid<float>;
		template class CustomHumanoid<double>;
	}
}

