
/**
  *
  * @file SpatialPoint.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	namespace Quantity {
		template <typename Scalar = double>
		class SpatialPoint final {
			private :
				static constexpr int Rank3 = 3;
				static constexpr int SpatialPointRank = Rank3 * 2;

			public :
				using Vector3 = Tools::Math::Vector<Scalar, Rank3>;
				using Vector6 = Tools::Math::Vector<Scalar, SpatialPointRank>;
				using Matrix3x3 = Tools::Math::Matrix<Scalar, Rank3, Rank3>;

				using Point = Vector3;
				using EulerAngles = Vector3;

				using Quaternion = Eigen::Quaternion<Scalar>;

				void reset();
				void reset_point();
				void reset_angle();

				const Point &point() const;
				SpatialPoint &point(const Point &);
				SpatialPoint &point(const Scalar &x, const Scalar &y, const Scalar &z);

				const EulerAngles &angle() const;
				SpatialPoint &angle(const EulerAngles &);
				SpatialPoint &angle(const Scalar &roll, const Scalar &pitch, const Scalar &yaw);

				const Quaternion &quaternion();
				SpatialPoint &quaternion(const Quaternion &);

				const Matrix3x3 &rotation();
				SpatialPoint &rotation(const Matrix3x3 &);

				void operator ()(const Vector6 &);
				const Vector6 operator ()();

				SpatialPoint &operator += (const SpatialPoint &);
				SpatialPoint &operator -= (const SpatialPoint &);

				bool operator == (const SpatialPoint &);
				bool operator != (const SpatialPoint &);

			private :
				Point point_vector;

				Quaternion q_rotation;

		};
	}
}

