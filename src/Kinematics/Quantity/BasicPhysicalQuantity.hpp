
/**
  *
  * @file BasicPhysicalQuantity.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

namespace Kinematics {
	namespace Quantity {
		template <typename Base>
		class BasicPhysicalQuantity {
			public :
				using BaseType = Base;

				BaseType &position() {
					return x;
				}

				const BaseType &position() const {
					return x;
				}

				BaseType &velocity() {
					return dx;
				}

				const BaseType &velocity() const {
					return dx;
				}

				BaseType &acceleration() {
					return ddx;
				}

				const BaseType &acceleration() const {
					return ddx;
				}

				BaseType &operator () () {
					return x;
				}

				const BaseType &operator () () const {
					return x;
				}

			protected :
				BaseType x,
						 dx,
						 ddx;

		};
	}
}

