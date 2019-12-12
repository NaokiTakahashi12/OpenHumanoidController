
#include "ModelSelector.hpp"

namespace Kinematics {
	namespace Model {
		template <typename Scalar>
		ModelSelector<Scalar>::ModelSelector() {
		}

		template <typename Scalar>
		ModelSelector<Scalar>::~ModelSelector() {
		}

		template <typename Scalar>
		typename ModelSelector<Scalar>::Ptr ModelSelector<Scalar>::make_ptr() {
			return std::make_unique<ModelSelector>();
		}

		template class ModelSelector<float>;
		template class ModelSelector<double>;
	}
}

