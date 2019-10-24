
#pragma once

#include "Humanoid.hpp"

#include <functional>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		template <typename Scalar>
		class CustomHumanoid : public Humanoid<Scalar> {
			private :
				using typename Humanoid<Scalar>::Vector;
				using typename Humanoid<Scalar>::EulerAngles;

			public :
				//using PatternFunctor = std::function<

				CustomHumanoid();
				~CustomHumanoid();

			private :

		};
	}
}

