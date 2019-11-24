
#include "MultipleIKSelector.hpp"

#include "LevenbergMarquardtForSerialLink.hpp"
#include "CustomMultipleIK.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		MultipleIKSelector<Scalar>::MultipleIKSelector(ModelPtr &new_model) {
			model = new_model;

			default_register();
		}

		template <typename Scalar>
		MultipleIKSelector<Scalar>::~MultipleIKSelector() {
		}

		template <typename Scalar>
		typename MultipleIKSelector<Scalar>::Ptr MultipleIKSelector<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<MultipleIKSelector<Scalar>>(new_model);
		}

		template <typename Scalar>
		void MultipleIKSelector<Scalar>::default_register() {
			this->register_object(LevenbergMarquardtForSerialLink<Scalar>::get_key(), LevenbergMarquardtForSerialLink<Scalar>::make_ptr(model));
			this->register_object(CustomMultipleIK<Scalar>::get_key(), CustomMultipleIK<Scalar>::make_ptr(model));
		}

		template class MultipleIKSelector<float>;
		template class MultipleIKSelector<double>;
	}
}

