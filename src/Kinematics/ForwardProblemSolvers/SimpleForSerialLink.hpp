
/**
  *
  * @file SimpleForSerialLink.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "MultipleFK.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		class SimpleForSerialLink : public MultipleFK<Scalar> {
			public :
				using Ptr = std::unique_ptr<SimpleForSerialLink>;

				using ModelPtr = ForwardProblemSolverBase::ModelPtr;

				SimpleForSerialLink(ModelPtr &);
				virtual ~SimpleForSerialLink();

				static const std::string get_key();

				static Ptr make_ptr(ModelPtr &);

				virtual bool compute() override;

		};
	}
}

