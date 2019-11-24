
#pragma once

#include "../ObjectSelector.hpp"

#include <string>

#include "MultipleIK.hpp"

namespace Kinematics {
	namespace InverseProblemSolvers {
		template <typename Scalar>
		class MultipleIKSelector : public ObjectSelector<MultipleIK<Scalar>, std::string> {
			public :
				using Ptr = std::unique_ptr<MultipleIKSelector>;

				using ModelPtr = typename MultipleIK<Scalar>::ModelPtr;

				MultipleIKSelector(ModelPtr &);

				virtual ~MultipleIKSelector();

				static Ptr make_ptr(ModelPtr &);

			private :
				ModelPtr model;

				void default_register();
		};
	}
}

