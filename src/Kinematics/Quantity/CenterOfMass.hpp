
/**
  *
  * @file CenterOfMass.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "BasicPhysicalQuantity.hpp"

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	namespace Quantity {
		template <typename Scaler>
		class CenterOfMass : public BasicPhysicalQuantity<Tools::Math::Vector3<Scaler>> {
			public :
				using Vector3 = typename BasicPhysicalQuantity<Tools::Math::Vector3<Scaler>>::BaseType;

				CenterOfMass();
				virtual ~CenterOfMass();

				void reset();

		};
	}
}

