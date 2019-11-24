
/**
  *
  * @file Parameters.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Parameters.hpp"

namespace Kinematics {
	template <typename Scalar>
	Parameters<Scalar>::Parameters(const DegreeOfFreedom &dof) {
		joint_angle_data = std::make_unique<Quantity::JointAngle<Scalar>>(dof);
		center_of_mass_data = std::make_unique<Quantity::CenterOfMass<Scalar>>();
		angular_momentum_data = std::make_unique<Quantity::AngularMomentum<Scalar>>();

		access_mutex_for_joint_angle = std::make_unique<std::mutex>();
		access_mutex_for_center_of_mass = std::make_unique<std::mutex>();
		access_mutex_for_angular_momentum = std::make_unique<std::mutex>();
	}

	template <typename Scalar>
	Parameters<Scalar>::Parameters(const Parameters<Scalar> &para) : Parameters(para.joint_angle_data->dof()) {
		*(this->joint_angle_data) = *(para.joint_angle_data);
		*(this->center_of_mass_data) = *(para.center_of_mass_data);
		*(this->angular_momentum_data) = *(para.angular_momentum_data);
	}

	template <typename Scalar>
	Parameters<Scalar>::~Parameters() {
	}

	template <typename Scalar>
	typename Parameters<Scalar>::Ptr Parameters<Scalar>::make_ptr(const DegreeOfFreedom &dof) {
		return std::make_shared<Parameters<Scalar>>(dof);
	}

	template <typename Scalar>
	Quantity::JointAngle<Scalar> &Parameters<Scalar>::joint_angle() {
		const auto lock = std::lock_guard<std::mutex>(*access_mutex_for_joint_angle);

		return *joint_angle_data;
	}

	template <typename Scalar>
	Quantity::CenterOfMass<Scalar> &Parameters<Scalar>::center_of_mass() {
		const auto lock = std::lock_guard<std::mutex>(*access_mutex_for_center_of_mass);

		return *center_of_mass_data;
	}

	template <typename Scalar>
	Quantity::AngularMomentum<Scalar> &Parameters<Scalar>::angular_momentum() {
		const auto lock = std::lock_guard<std::mutex>(*access_mutex_for_angular_momentum);

		return *angular_momentum_data;
	}

	template <typename Scalar>
	Parameters<Scalar> &Parameters<Scalar>::operator = (const Parameters &para) {
		if(this != &para) {
			*(this->joint_angle_data) = *(para.joint_angle_data);
			*(this->center_of_mass_data) = *(para.center_of_mass_data);
			*(this->angular_momentum_data) = *(para.angular_momentum_data);
		}

		return *this;
	}

	template class Parameters<float>;
	template class Parameters<double>;
	template class Parameters<long double>;
}

