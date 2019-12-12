
/**
  *
  * @file CustomMultipleIK.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "MultipleIK.hpp"

#include <functional>

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		class CustomMultipleIK : public MultipleIK<Scalar> {
			public :
				using Ptr = std::unique_ptr<CustomMultipleIK>;

				using ModelPtr = InverseProblemSolverBase::ModelPtr;

				using SolverFunction = std::function<
					bool(ModelPtr &, typename Parameters<Scalar>::Ptr &, typename ControlPointMap<Scalar>::Ptr &)
				>;

				CustomMultipleIK(ModelPtr &);

				virtual ~CustomMultipleIK();

				static Ptr make_ptr(ModelPtr &);

				static const std::string get_key();

				bool compute() override;
				bool compute(const Quantity::JointAngle<Scalar> &) override;

				void register_solver_function(SolverFunction);

			private :
				SolverFunction solver_function;

		};
	}
}

