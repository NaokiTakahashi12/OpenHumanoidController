
/**
  *
  * @file SpatialPoint.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SpatialPoint.hpp"

#include <sstream>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

namespace Kinematics {
	namespace Quantity {
		template <typename Scalar>
		void SpatialPoint<Scalar>::reset() {
			reset_point();
			reset_angle();
		}

		template <typename Scalar>
		void SpatialPoint<Scalar>::reset_point() {
			point_vector = Point::Zero();
		}

		template <typename Scalar>
		void SpatialPoint<Scalar>::reset_angle() {
			q_rotation.vec() = Vector3::Zero();
			q_rotation.w() = 0;
		}

		template <typename Scalar>
		const typename SpatialPoint<Scalar>::Point &SpatialPoint<Scalar>::point() const {
			return point_vector;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::point(const Point &new_point) {
			point_vector = new_point;

			return *this;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::point(const Scalar &x, const Scalar &y, const Scalar &z) {
			point_vector << x, y, z;

			return *this;
		}

		template <typename Scalar>
		const typename SpatialPoint<Scalar>::EulerAngles &SpatialPoint<Scalar>::angle() const {
			static Eigen::EulerAngles<Scalar, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> euler_angle_system;

			euler_angle_system = q_rotation;

			return euler_angle_system.angles();
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::angle(const EulerAngles &new_angle) {
			q_rotation = Eigen::AngleAxis<Scalar>(new_angle.x(), EulerAngles::UnitX());
			q_rotation = q_rotation * Eigen::AngleAxis<Scalar>(new_angle.y(), EulerAngles::UnitY());
			q_rotation = q_rotation * Eigen::AngleAxis<Scalar>(new_angle.z(), EulerAngles::UnitZ());

			return *this;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::angle(const Scalar &roll, const Scalar &pitch, const Scalar &yaw) {
			q_rotation = Eigen::AngleAxis<Scalar>(roll, EulerAngles::UnitX());
			q_rotation = q_rotation * Eigen::AngleAxis<Scalar>(pitch, EulerAngles::UnitY());
			q_rotation = q_rotation * Eigen::AngleAxis<Scalar>(yaw, EulerAngles::UnitZ());

			return *this;
		}

		template <typename Scalar>
		const typename SpatialPoint<Scalar>::Quaternion &SpatialPoint<Scalar>::quaternion() {
			return q_rotation;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::quaternion(const Quaternion &quaternion) {
			q_rotation = quaternion;

			return *this;
		}

		template <typename Scalar>
		const typename SpatialPoint<Scalar>::Matrix3x3 &SpatialPoint<Scalar>::rotation() {
			static Matrix3x3 rotation_matrix;

			rotation_matrix = q_rotation.matrix();

			return rotation_matrix;

		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::rotation(const Matrix3x3 &rotation_matrix) {
			const auto rrt = rotation_matrix * rotation_matrix.transpose();

			if(1e-5 < (Matrix3x3::Identity() - rrt).norm()) {
				std::stringstream ss;

				ss << "Input Matrix is\n" << rotation_matrix << "\n";
				ss << "\nM*MT is\n" << rrt << "\n";
				ss << (Matrix3x3::Identity() - rrt).norm() << "\n";
				ss << "\n";

				throw std::runtime_error("Unknown type matrix from Kinematics::SpatialPoint\n" + ss.str());
			}

			q_rotation = rotation_matrix;

			return *this;
		}

		template <typename Scalar>
		void SpatialPoint<Scalar>::operator ()(const Vector6 &new_point_with_angle_vector) {
			point(new_point_with_angle_vector.block(0, 0, Rank3, 1));
			angle(new_point_with_angle_vector.block(Rank3, 0, Rank3, 1));
		}

		template <typename Scalar>
		const typename SpatialPoint<Scalar>::Vector6 SpatialPoint<Scalar>::operator ()() {
			static Vector6 point_with_angle_vector;

			point_with_angle_vector << point(), angle();

			return point_with_angle_vector;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::operator += (const SpatialPoint<Scalar> &spatial_point) {
			if(this != &spatial_point) {
				static Eigen::EulerAngles<Scalar, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> euler_angle_system;

				euler_angle_system = spatial_point.q_rotation;

				this->angle(this->angle() + euler_angle_system.angles());

				this->point_vector += spatial_point.point_vector;
			}

			return *this;
		}

		template <typename Scalar>
		SpatialPoint<Scalar> &SpatialPoint<Scalar>::operator -= (const SpatialPoint<Scalar> &spatial_point) {
			if(this != &spatial_point) {
				static Eigen::EulerAngles<Scalar, Eigen::EulerSystem<Eigen::EULER_X, Eigen::EULER_Y, Eigen::EULER_Z>> euler_angle_system;

				euler_angle_system = spatial_point.q_rotation;

				this->angle(this->angle() - euler_angle_system.angles());

				this->point_vector -= spatial_point.point_vector;
			}

			return *this;
		}

		template <typename Scalar>
		bool SpatialPoint<Scalar>::operator == (const SpatialPoint &spatial_point) {
			return 
				this->point_vector == spatial_point.point_vector
				&& this->q_rotation.vec() == spatial_point.q_rotation.vec()
				&& this->q_rotation.w() == spatial_point.q_rotation.w();
		}

		template <typename Scalar>
		bool SpatialPoint<Scalar>::operator != (const SpatialPoint &spatial_point) {
			return !this->operator == (spatial_point);
		}

		template class SpatialPoint<float>;
		template class SpatialPoint<double>;
		template class SpatialPoint<long double>;
	}
}

