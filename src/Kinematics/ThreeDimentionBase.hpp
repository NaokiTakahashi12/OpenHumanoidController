
/**
  *
  * @file ThreeDimentionBase.hpp
  * @author Naoki Takahashi
  *
  **/

#include <array>

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	class ThreeDimentionBase {
		public :
			ThreeDimentionBase();

		protected :
			//! Centor point = 0, X shift = 1, Y shift = 2
			using GoalOrbit = std::array<Tools::Math::Vector3<double>, 3>;

	};
}

