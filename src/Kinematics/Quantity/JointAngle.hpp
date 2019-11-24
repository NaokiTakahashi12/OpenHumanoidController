
/**
  *
  * @file JointAngle.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "BasicPhysicalQuantity.hpp"

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	namespace Quantity {
		template <typename Scalar>
		class JointAngle : public BasicPhysicalQuantity<Tools::Math::VectorX<Scalar>> {
			public :
				using DegreeOfFreedom = unsigned int;

				using VectorN = typename BasicPhysicalQuantity<Tools::Math::VectorX<Scalar>>::BaseType;

				JointAngle(const DegreeOfFreedom &rank);

				JointAngle(const JointAngle &);

				virtual ~JointAngle();

				const DegreeOfFreedom &dof() const;

				bool check() const;

				void size_assert() const;

				void reset();
				void reset_with_size_assert();

				JointAngle &operator = (const JointAngle &);

			private :
				const DegreeOfFreedom rank;

		};
	}
}

