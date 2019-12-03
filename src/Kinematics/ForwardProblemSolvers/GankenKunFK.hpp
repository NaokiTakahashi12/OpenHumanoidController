
/**
  *
  * @file GankenKunFK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#pragma once

#include "MultipleFK.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		class GankenKunFK : public MultipleFK<Scalar> {
			public :
				using Ptr = std::unique_ptr<GankenKunFK>;

				using ModelPtr = ForwardProblemSolverBase::ModelPtr;

				GankenKunFK(ModelPtr &);
				virtual ~GankenKunFK();

				static const std::string get_key();

				static Ptr make_ptr(ModelPtr &);

				bool compute() override;

		};
	}
}

