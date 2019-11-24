
/**
  *
  * @file InverseProblemSolverBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "InverseProblemSolverBase.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		InverseProblemSolverBase::InverseProblemSolverBase(ModelPtr &model_ptr) {
			model = model_ptr;
		}

		InverseProblemSolverBase::~InverseProblemSolverBase() {
		}
	}
}

