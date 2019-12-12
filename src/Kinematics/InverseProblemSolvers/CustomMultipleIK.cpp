
/**
  *
  * @file CustomMultipleIK.cpp
  * @author Naoki Takahashi
  *
  **/

#include "CustomMultipleIK.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		CustomMultipleIK<Scalar>::CustomMultipleIK(ModelPtr &new_model) : MultipleIK<Scalar>(new_model) {
		}

		template <typename Scalar>
		CustomMultipleIK<Scalar>::~CustomMultipleIK() {
		}

		template <typename Scalar>
		typename CustomMultipleIK<Scalar>::Ptr CustomMultipleIK<Scalar>::make_ptr(ModelPtr &new_model) {
			return std::make_unique<CustomMultipleIK<Scalar>>(new_model);
		}

		template <typename Scalar>
		const std::string CustomMultipleIK<Scalar>::get_key() {
			return "CustomMultipleIK";
		}

		template <typename Scalar>
		bool CustomMultipleIK<Scalar>::compute() {
			if(!solver_function) {
				throw std::runtime_error("Can not call solve_function from Kinematics::InverseProblemSolvers::CustomMultipleIK");
			}

			return solver_function(this->model, this->parameters, this->control_point_map);
		}

		template <typename Scalar>
		bool CustomMultipleIK<Scalar>::compute(const Quantity::JointAngle<Scalar> &init_q) {
			if(!solver_function) {
				throw std::runtime_error("Can not call solve_function from Kinematics::InverseProblemSolvers::CustomMultipleIK");
			}

			this->parameters->joint_angle()() = init_q();

			const bool is_success = solver_function(this->model, this->parameters, this->control_point_map);

			if(this->parameters->joint_angle()() == init_q() && !is_success) {
				this->parameters->joint_angle()() -= this->parameters->joint_angle()();
			}

			return is_success;
		}

		template <typename Scalar>
		void CustomMultipleIK<Scalar>::register_solver_function(SolverFunction new_solver_function) {
			if(!new_solver_function) {
				throw std::runtime_error("Failed registration solve_function from Kinematics::InverseProblemSolvers::CustomMultipleIK");
			}

			solver_function = new_solver_function;
		}

		template class CustomMultipleIK<float>;
		template class CustomMultipleIK<double>;
	}
}

