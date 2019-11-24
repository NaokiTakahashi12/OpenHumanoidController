
/**
  *
  * @file InverseProblemSolverBase.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>

#include "../Model/RBDLBased.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		class InverseProblemSolverBase {
			public :
				using ModelPtr = Model::RBDLBased::Ptr;
				
				InverseProblemSolverBase(ModelPtr &);
				virtual ~InverseProblemSolverBase();

				virtual bool compute() = 0;

			protected :
				ModelPtr model;
		};
	}
}

