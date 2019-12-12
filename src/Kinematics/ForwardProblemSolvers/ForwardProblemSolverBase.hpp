
/**
  *
  * @file ForwardProblemSolverBase.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../Model/RBDLBased.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		class ForwardProblemSolverBase {
			public :
				using ModelPtr = Model::RBDLBased::Ptr;

				ForwardProblemSolverBase(ModelPtr &);
				virtual ~ForwardProblemSolverBase();

				virtual bool compute() = 0;

			protected :
				ModelPtr model;

		};
	}
}

