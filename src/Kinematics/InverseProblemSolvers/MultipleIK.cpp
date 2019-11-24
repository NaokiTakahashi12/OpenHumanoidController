
/**
  *
  * @file MultipleIK.cpp
  * @author Naoki Takahashi
  *
  **/

#include "MultipleIK.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		MultipleIK<Scalar>::MultipleIK(ModelPtr &model_ptr) : InverseProblemSolverBase(model_ptr) {
		}

		template <typename Scalar>
		MultipleIK<Scalar>::~MultipleIK() {
		}

		template <typename Scalar>
		void MultipleIK<Scalar>::register_map(ControlPointMapPtr &map) {
			if(!map) {
				throw std::runtime_error("Can not access to map from Kinematics::InverseProblemSolvers::MultipleIK");
			}

			control_point_map = map;
		}

		template <typename Scalar>
		void MultipleIK<Scalar>::register_parameters(ParametersPtr &para) {
			if(!para) {
				throw std::runtime_error("Can not access to parameters from Kinematics::InverseProblemSolvers::MultipleIK");
			}

			parameters = para;
		}

		template class MultipleIK<float>;
		template class MultipleIK<double>;
	}
}
