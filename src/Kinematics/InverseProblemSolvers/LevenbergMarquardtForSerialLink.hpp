
/**
  *
  * @file LevenbergMarquardtForSerialLink.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once 

#include "InverseProblemSolverBase.hpp"
#include "MultipleIK.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		class LevenbergMarquardtForSerialLink : public MultipleIK<Scalar> {
			protected :
				using typename InverseProblemSolverBase::ModelPtr;

			public :
				using Ptr = std::unique_ptr<LevenbergMarquardtForSerialLink>;

				LevenbergMarquardtForSerialLink(ModelPtr &);
				virtual ~LevenbergMarquardtForSerialLink();

				static const std::string get_key();

				static Ptr make_ptr(ModelPtr &);

				virtual bool compute() override final;
				virtual bool compute(const Quantity::JointAngle<Scalar> &) override final;
		};
	}
}
