
/**
  *
  * @file MultipleFKSelector.cpp
  * @author Naoki Takahashi
  *
  **/

#include "MultipleFKSelector.hpp"

#include "SimpleForSerialLink.hpp"
#include "CustomMultipleFK.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		MultipleFKSelector<Scalar>::MultipleFKSelector(ModelPtr &new_model) {
			model = new_model;

			default_register();
		}

		template <typename Scalar>
		MultipleFKSelector<Scalar>::~MultipleFKSelector() {
		}

		template <typename Scalar>
		typename MultipleFKSelector<Scalar>::Ptr MultipleFKSelector<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<MultipleFKSelector>(new_model);
		}

		template <typename Scalar>
		void MultipleFKSelector<Scalar>::default_register() {
			this->register_object(SimpleForSerialLink<Scalar>::get_key(), SimpleForSerialLink<Scalar>::make_ptr(model));
			this->register_object(CustomMultipleFK<Scalar>::get_key(), CustomMultipleFK<Scalar>::make_ptr(model));
		}

		template class MultipleFKSelector<float>;
		template class MultipleFKSelector<double>;
	}
}

