
/**
  *
  * @file GankenKunIK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#pragma once

#include "InverseProblemSolverBase.hpp"
#include "MultipleIK.hpp"
#include "../Model/Hardware/GankenKun2018/Kinematics.h"
#include "../Model/Hardware/GankenKun2018/Link.h"
#include "../Model/Hardware/GankenKun2018/Constant.h"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		class GankenKunIK : public MultipleIK<Scalar> {
			protected:
				using ModelPtr = InverseProblemSolverBase::ModelPtr;

			public :
				using Ptr = std::unique_ptr<GankenKunIK>;
				
				GankenKunIK(ModelPtr &);
				virtual ~GankenKunIK();

				static const std::string get_key();

				static Ptr make_ptr(ModelPtr &);

				virtual bool compute() override final;
				virtual bool compute(const Quantity::JointAngle<Scalar> &) override final;

			private:
				GankenKun2018::Kinematics *kine;
				GankenKun2018::Link *link;
				float servo_angle[GankenKun2018::Const::SERVO_NUM];
				bool is_first_compute = true;
		};
	}
}

