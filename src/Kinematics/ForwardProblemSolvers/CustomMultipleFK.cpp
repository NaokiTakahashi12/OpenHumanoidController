
/**
  *
  * @file CustomMultipleFK.cpp
  * @author Naoki Takahashi
  *
  **/

#include "CustomMultipleFK.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		CustomMultipleFK<Scalar>::CustomMultipleFK(ModelPtr &new_model) : MultipleFK<Scalar>(new_model) {
		}

		template <typename Scalar>
		CustomMultipleFK<Scalar>::~CustomMultipleFK() {
		}

		template <typename Scalar>
		typename CustomMultipleFK<Scalar>::Ptr CustomMultipleFK<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<CustomMultipleFK<Scalar>>(new_model);
		}

		template <typename Scalar>
		const std::string CustomMultipleFK<Scalar>::get_key() {
			return "CustomMultipleFK";
		}

		template <typename Scalar>
		bool CustomMultipleFK<Scalar>::compute() {
			if(!solver_function) {
				throw std::runtime_error("Can not call solver_function from Kinematics::ForwardProblemSolvers::CustomMultipleFK");
			}

			return solver_function(this->model, this->parameters, this->control_point_map);
		}

		template <typename Scalar>
		void CustomMultipleFK<Scalar>::register_solver_function(SolverFunction new_solverr_function) {
			if(!new_solverr_function) {
				throw std::runtime_error("Failed registration solver_function from Kinematics::ForwardProblemSolvers::CustomMultipleFK");
			}

			solver_function = new_solverr_function;
		}

		template class CustomMultipleFK<float>;
		template class CustomMultipleFK<double>;
	}
}

