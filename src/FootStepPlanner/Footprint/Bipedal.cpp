
/**
  *
  * @file Bipedal.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Bipedal.hpp"

namespace FootStepPlanner {
	namespace Footprint {
		template <typename Scalar>
		typename Bipedal<Scalar>::Ptr Bipedal<Scalar>::make_ptr() {
			return std::make_unique<Bipedal<Scalar>>();
		}

		template struct Bipedal<float>;
		template struct Bipedal<double>;
	}
}

