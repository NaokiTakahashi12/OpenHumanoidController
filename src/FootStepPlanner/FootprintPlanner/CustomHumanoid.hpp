
/**
  *
  * @file CustomHumanoid.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "Humanoid.hpp"

#include <functional>

namespace FootStepPlanner {
	namespace FootprintPlanner {
		//! @todo Design using configuration file.
		template <typename Scalar>
		class CustomHumanoid : public Humanoid<Scalar> {
			public :
				using Ptr = std::unique_ptr<CustomHumanoid>;

				CustomHumanoid();
				~CustomHumanoid();

				static std::string get_key();
				static Ptr make_ptr();

				void full_step() override;

			private :

		};
	}
}

