
/**
  *
  * @file Model.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Model.hpp"

#include <stdexcept>

#include <rbdl/addons/urdfreader/urdfreader.h>

namespace Kinematics {
	Model::Model() {
	}

	void Model::create_model_from_urdf(const std::string &filename) {
		create_core_model();
		auto is_read = RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), core_model.get(), false);
		if(!is_read) {
			throw std::runtime_error("Can not read URDF file: " + filename);
		}
	}

	const Model::RawModel Model::get_raw() {
		if(!core_model) {
			throw std::runtime_error("Not exist raw model from Kinematics::Model");
		}
		return *core_model;
	}

	void Model::create_core_model() {
		if(core_model) {
			throw std::runtime_error("Already created core model from Kinematics::Model");
		}
		core_model = std::make_unique<RawModel>();
	}
}

