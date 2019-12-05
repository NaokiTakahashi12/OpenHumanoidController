
/**
  *
  * @file GankenKunFK.hpp
  * @author Yasuo Hayashibara
  *
  **/

#pragma once

#include "MultipleFK.hpp"
#include "../Hardware/GankenKun2018/Kinematics.h"
#include "../Hardware/GankenKun2018/Link.h"
#include "../Hardware/GankenKun2018/Constant.h"

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

			private:
				GankenKun2018::Kinematics *kine;
				GankenKun2018::Link *link;
				float servo_angle[GankenKun2018::Const::SERVO_NUM];
		};
	}
}

