
/**
  *
  * @file MultipleIK.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "InverseProblemSolverBase.hpp"

#include "../Parameters.hpp"
#include "../ControlPointMap.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		class MultipleIK : public InverseProblemSolverBase {
			protected :
				using ParametersPtr = typename Parameters<Scalar>::Ptr;
				using ControlPointMapPtr = typename ControlPointMap<Scalar>::Ptr;

			public :
				using InverseProblemSolverBase::ModelPtr;

				using Ptr = std::unique_ptr<MultipleIK>;

				MultipleIK(ModelPtr &);
				virtual ~MultipleIK();

				virtual bool compute() = 0;
				virtual bool compute(const Quantity::JointAngle<Scalar> &) = 0;

				void register_map(ControlPointMapPtr &);
				void register_parameters(ParametersPtr &);

			protected :
				ParametersPtr parameters;

				ControlPointMapPtr control_point_map;
		};
	}
}

