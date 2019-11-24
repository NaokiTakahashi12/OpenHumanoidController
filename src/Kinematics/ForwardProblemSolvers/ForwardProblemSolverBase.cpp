
/**
  *
  * @file ForwardProblemSolverBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ForwardProblemSolverBase.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		ForwardProblemSolverBase::ForwardProblemSolverBase(ModelPtr &new_model) {
			model = new_model;
		}

		ForwardProblemSolverBase::~ForwardProblemSolverBase() {
		}
	}
}

