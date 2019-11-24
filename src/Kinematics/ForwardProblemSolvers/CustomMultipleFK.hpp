
/**
  *
  * @file CustomMultipleFK.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "MultipleFK.hpp"

#include <functional>

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		class CustomMultipleFK : public MultipleFK<Scalar> {
			public :
				using ModelPtr = ForwardProblemSolverBase::ModelPtr;

				using Ptr = std::unique_ptr<CustomMultipleFK>;

				using SolverFunction = std::function<
					bool(ModelPtr &, typename Parameters<Scalar>::Ptr &, typename ControlPointMap<Scalar>::Ptr &)
				>;

				CustomMultipleFK(ModelPtr &new_model);

				virtual ~CustomMultipleFK();

				static Ptr make_ptr(ModelPtr &);

				static const std::string get_key();

				bool compute() override;

				void register_solver_function(SolverFunction);

			private :
				SolverFunction solver_function;

		};
	}
}

