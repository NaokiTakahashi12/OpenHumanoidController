
/**
  *
  * @file MultipleFKSelector.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../ObjectSelector.hpp"

#include <string>

#include "MultipleFK.hpp"

namespace Kinematics {
	namespace ForwardProblemSolvers {
		template <typename Scalar>
		class MultipleFKSelector : public ObjectSelector<MultipleFK<Scalar>, std::string> {
			public :
				using Ptr = std::unique_ptr<MultipleFKSelector>;

				using ModelPtr = typename MultipleFK<Scalar>::ModelPtr;

				MultipleFKSelector(ModelPtr &);

				virtual ~MultipleFKSelector();

				static Ptr make_ptr(ModelPtr &);

			private :
				ModelPtr model;

				void default_register();

		};
	}
}

