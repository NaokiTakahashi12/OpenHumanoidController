
/**
  *
  * @file MultipleFK.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "ForwardProblemSolverBase.hpp"

#include <Tools/Math/Matrix.hpp>

#include "../Parameters.hpp"
#include "../ControlPointMap.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		class MultipleFK : public ForwardProblemSolverBase {
			protected :
				using ParametersPtr = typename Parameters<Scalar>::Ptr;
				using ControlPointMapPtr = typename ControlPointMap<Scalar>::Ptr;

			public :
				using Ptr = std::unique_ptr<MultipleFK>;

				using ForwardProblemSolverBase::ModelPtr;

				using VectorX = Tools::Math::VectorX<Scalar>;

				MultipleFK(ModelPtr &);
				virtual ~MultipleFK();

				void register_map(ControlPointMapPtr &);
				void register_parameters(ParametersPtr &);

			protected :
				ParametersPtr parameters;

				ControlPointMapPtr control_point_map;

				VectorX q;
		};
	}
}

