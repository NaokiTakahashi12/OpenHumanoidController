
/**
  *
  * @file AngularMomentum.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	namespace Quantity {
		template <typename Scaler>
		class AngularMomentum {
			public :
				using Vector3 = Tools::Math::Vector3<Scaler>;

				AngularMomentum();

				virtual ~AngularMomentum();

				void reset();

				const Vector3 &change() const;
				Vector3 &change();

				const Vector3 &operator () () const;
				Vector3 &operator () ();

			private :
				Vector3 angular_momentum_vector,
						change_of_angular_momentum;

		};
	}
}

