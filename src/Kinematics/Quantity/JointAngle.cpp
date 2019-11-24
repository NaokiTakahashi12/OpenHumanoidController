
/**
  *
  * @file JointAngle.cpp
  * @author Naoki Takahashi
  *
  **/

#include "JointAngle.hpp"

#include <stdexcept>

namespace Kinematics {
	namespace Quantity {
		template <typename Scalar>
		JointAngle<Scalar>::JointAngle(const DegreeOfFreedom &rank) : rank(rank) {
			this->x = VectorN::Zero(rank);
			this->dx = VectorN::Zero(rank);
			this->ddx = VectorN::Zero(rank);
		}

		template <typename Scalar>
		JointAngle<Scalar>::JointAngle(const JointAngle<Scalar> &joint_angle) : JointAngle(joint_angle.rank) {
			this->x = joint_angle.x;
			this->dx = joint_angle.dx;
			this->ddx = joint_angle.ddx;
		}

		template <typename Scalar>
		JointAngle<Scalar>::~JointAngle() {
		}

		template <typename Scalar>
		const typename JointAngle<Scalar>::DegreeOfFreedom &JointAngle<Scalar>::dof() const {
			return rank;
		}

		template <typename Scalar>
		bool JointAngle<Scalar>::check() const {
			if(rank != this->x.size()) {
				return false;
			}
			else if(rank != this->dx.size()) {
				return false;
			}
			else if(rank != this->ddx.size()) {
				return false;
			}

			return true;
		}

		template <typename Scalar>
		void JointAngle<Scalar>::size_assert() const {
			if(!check()) {
				throw std::runtime_error("Assert of different Q space size from Kinematics::JointAngle");
			}
		}

		template <typename Scalar>
		void JointAngle<Scalar>::reset() {
			this->x = VectorN::Zero(rank);
			this->dx = VectorN::Zero(rank);
			this->ddx = VectorN::Zero(rank);
		}

		template <typename Scalar>
		void JointAngle<Scalar>::reset_with_size_assert() {
			size_assert();
			reset();
		}

		template <typename Scalar>
		JointAngle<Scalar> &JointAngle<Scalar>::operator = (const JointAngle<Scalar> &joint_angle) {
			if(this != &joint_angle) {
				if(this->rank != joint_angle.rank) {
					throw std::runtime_error("Failed Q copy from Kinematics::JointAngle");
				}

				this->x = joint_angle.x;
				this->dx = joint_angle.dx;
				this->ddx = joint_angle.ddx;
			}

			return *this;
		}

		template class JointAngle<float>;
		template class JointAngle<double>;
		template class JointAngle<long double>;
	}
}

