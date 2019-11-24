
/**
  *
  * @file RBDLBased.cpp
  * @author Naoki Takahashi
  *
  **/

#include "RBDLBased.hpp"

#include <stdexcept>

#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace Kinematics {
	namespace Model {
		RBDLBased::RBDLBased(const std::string &new_model_file_name) : ModelBase(new_model_file_name) {
		}

		RBDLBased::~RBDLBased() {
		}

		void RBDLBased::initialize() {
			load_model_from_file(model_file());

			if(!basic_model) {
				throw std::runtime_error("Failed model construction from Kinematics::Model::RBDLBased");
			}

			joint_angle = RigidBodyDynamics::Math::VectorNd::Zero(basic_model->dof_count);
			joint_speed = RigidBodyDynamics::Math::VectorNd::Zero(basic_model->dof_count);
			joint_acceleration = RigidBodyDynamics::Math::VectorNd::Zero(basic_model->dof_count);

			update_com_status();
		}

		unsigned int RBDLBased::dof() const {
			return basic_model->dof_count;
		}

		RBDLBased::BodyID RBDLBased::body_id(const std::string &string_key) {
			const BodyID body_id = access().GetBodyId(string_key.c_str());

			if(!access().IsBodyId(body_id)) {
				throw std::runtime_error("Not find body id: " + string_key + " from Kinematics::Model::RBDLBased");
			}

			return body_id;
		}

		bool RBDLBased::is_body_id(const BodyID &body_id) {
			return access().IsBodyId(body_id);
		}

		bool RBDLBased::is_fixed_body(const BodyID &body_id) {
			return access().IsFixedBodyId(body_id);
		}

		bool RBDLBased::is_fixed_body(const std::string &string_key) {
			return is_fixed_body(body_id(string_key));
		}

		void RBDLBased::load_model_from_file(const std::string &load_model_from_file) {
			if(basic_model) {
				basic_model.reset();
			}
			basic_model = std::make_unique<BasicModel>();

			const bool is_read = RigidBodyDynamics::Addons::URDFReadFromFile(
					load_model_from_file.c_str(),
					basic_model.get(),
					false,
					false
			);

			if(!is_read) {
				throw std::runtime_error("Can not read model file from Kinematics::Model::RBDLBased");
			}
		}

		void RBDLBased::update_com_status() {
			RigidBodyDynamics::Utils::CalcCenterOfMass(
				this->access(),
				joint_angle,
				joint_speed,
				&joint_acceleration,
				this->kg_mass,
				center_of_mass,
				&center_of_mass_velocity,
				&center_of_mass_acceleration,
				&center_of_mass_anguler_momentum,
				&center_of_mass_change_of_anguler_momentum,
				true
			);
		}

		RBDLBased::BasicModel &RBDLBased::access() {
			const auto lock = std::lock_guard<std::mutex>(get_access_model_mutex());

			if(!basic_model) {
				throw std::runtime_error("Can not access basic_model from Kinematics::Model::RBDLBased");
			}

			return *(basic_model.get());
		}
	}
}

