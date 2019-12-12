
/**
  *
  * @file Parameters.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <mutex>
#include <memory>

#include "Quantity/JointAngle.hpp"
#include "Quantity/CenterOfMass.hpp"
#include "Quantity/AngularMomentum.hpp"

namespace Kinematics {
	template <typename Scalar>
	class Parameters {
		public :
			using Ptr = std::shared_ptr<Parameters>;

			using DegreeOfFreedom = unsigned int;

			Parameters(const DegreeOfFreedom &);

			Parameters(const Parameters &);

			virtual ~Parameters();

			static Ptr make_ptr(const DegreeOfFreedom &);

			Quantity::JointAngle<Scalar> &joint_angle();

			Quantity::CenterOfMass<Scalar> &center_of_mass();

			Quantity::AngularMomentum<Scalar> &angular_momentum();

			Parameters &operator = (const Parameters &);

		private :
			std::unique_ptr<std::mutex> access_mutex_for_joint_angle;
			std::unique_ptr<std::mutex> access_mutex_for_center_of_mass;
			std::unique_ptr<std::mutex> access_mutex_for_angular_momentum;

			std::unique_ptr<Quantity::JointAngle<Scalar>> joint_angle_data;
			std::unique_ptr<Quantity::CenterOfMass<Scalar>> center_of_mass_data;
			std::unique_ptr<Quantity::AngularMomentum<Scalar>> angular_momentum_data;

	};
}

