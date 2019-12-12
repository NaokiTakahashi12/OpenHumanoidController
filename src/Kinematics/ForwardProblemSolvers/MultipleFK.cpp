
/**
  *
  * @file MultipleFK.hpp
  * @author Naoki Takahashi
  *
  **/

#include "MultipleFK.hpp"

#include <stdexcept>

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		MultipleFK<Scalar>::MultipleFK(ModelPtr &new_model) : ForwardProblemSolverBase(new_model) { 
		}

		template <typename Scalar>
		MultipleFK<Scalar>::~MultipleFK() {
		}

		template <typename Scalar>
		void MultipleFK<Scalar>::register_map(ControlPointMapPtr &map) {
			if(!map) {
				throw std::runtime_error("Can not access to map from Kinematics::ForwardProblemSolvers::MultipleFK");
			}

			control_point_map = map;
		}

		template <typename Scalar>
		void MultipleFK<Scalar>::register_parameters(ParametersPtr &para) {
			if(!para) {
				throw std::runtime_error("Can not access to parameters from Kinematics::ForwardProblemSolvers::MultipleFK");
			}

			parameters = para;
		}

		template class MultipleFK<float>;
		template class MultipleFK<double>;
	}
}
