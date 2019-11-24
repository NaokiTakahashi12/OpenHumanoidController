
#pragma once

#include <memory>

#include "RBDLBased.hpp"

namespace Kinematics {
	namespace Model {
		template <typename Scalar>
		class ModelSelector {
			public :
				using Ptr = std::unique_ptr<ModelSelector>;

				ModelSelector();
				~ModelSelector();

				static Ptr make_ptr();

			private :

		};
	}
}

