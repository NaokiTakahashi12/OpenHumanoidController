
/**
  *
  * @file RBDLBased.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "ModelBase.hpp"

#include <rbdl/Model.h>

#include <Tools/Math/Matrix.hpp>

namespace Kinematics {
	namespace Model {
		class RBDLBased : public ModelBase {
			protected :
				using BasicModel = RigidBodyDynamics::Model;
				using BasicModelPtr = std::unique_ptr<BasicModel>;

			public :
				using Ptr = std::shared_ptr<RBDLBased>;

				using BodyID = unsigned int;
				using Scalar = double;

				using VectorN = Tools::Math::VectorX<Scalar>;
				using Vector3 = Tools::Math::Vector3<Scalar>;

				RBDLBased(const std::string &new_model_file_name);
				virtual ~RBDLBased();

				virtual void initialize() override;

				unsigned int dof() const;

				BodyID body_id(const std::string &);

				bool is_body_id(const BodyID &);

				bool is_fixed_body(const BodyID &);
				bool is_fixed_body(const std::string &);

				BasicModel &access();

			protected :
				BasicModelPtr basic_model;

				Scalar kg_mass;

				RigidBodyDynamics::Math::VectorNd joint_angle,
												  joint_speed,
												  joint_acceleration;

				RigidBodyDynamics::Math::Vector3d center_of_mass,
												  center_of_mass_velocity,
												  center_of_mass_acceleration,
												  center_of_mass_anguler_momentum,
												  center_of_mass_change_of_anguler_momentum;

				void load_model_from_file(const std::string &load_model_file) override final;

				void update_com_status();
		};
	}
}

