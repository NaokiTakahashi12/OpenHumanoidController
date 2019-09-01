
/**
  *
  * @file Model.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Model.hpp"

#include <stdexcept>
#include <limits>

#include <rbdl/addons/urdfreader/urdfreader.h>

namespace Kinematics {
	Model::Model() {
	}

	Model::Model(const std::string &urdf_filename) {
		setup_from_urdf(urdf_filename);
	}

	void Model::setup_from_urdf(const std::string &filename) {
		create_model_from_urdf(filename);
		create_vectors_from_model_dof();
	}

	bool Model::is_load_model() {
		return core_model.operator bool ();
	}

	const Model::RawModel Model::raw() {
		if(!core_model) {
			throw std::runtime_error("Can not return raw model from Kinematics::Model");
		}
		return *core_model;
	}
	Model::RawModel &Model::access() {
		if(!core_model) {
			throw std::runtime_error("Can not access raw model from Kinematics::Model");
		}
		return *core_model;
	}

	 Model::VectorN Model::get_q() {
		if(!q) {
			throw std::runtime_error("Can not return Q from Kinematics::Model");
		}
		return *q;
	}

	Model::VectorN Model::get_qd() {
		if(!qd) {
			throw std::runtime_error("Can not return QDof from Kinematics::Model");
		}
		return *qd;
	}

	Model::VectorN Model::get_qdd() {
		if(!qdd) {
			throw std::runtime_error("Can not return QDDof from Kinematics::Model");
		}
		return *qdd;
	}

	Model::VectorN Model::get_tau() {
		if(!tau) {
			throw std::runtime_error("Can not return Tau from Kinematics::Model");
		}
		return *tau;
	}

	Model::VectorN &Model::access_q() {
		if(!q) {
			throw std::runtime_error("Can not access Q from Kinematics::Model");
		}
		return *q;
	}

	Model::VectorN &Model::access_qd() {
		if(!qd) {
			throw std::runtime_error("Can not access dQ from Kinematics::Model");
		}
		return *qd;
	}

	Model::VectorN &Model::access_qdd() {
		if(!qdd) {
			throw std::runtime_error("Can not access ddQ from Kinematics::Model");
		}
		return *qdd;
	}

	Model::VectorN &Model::access_tau() {
		if(!tau) {
			throw std::runtime_error("Can not access Tau from Kinematics::Model");
		}
		return *tau;
	}

	Model::DofSize Model::number_of_dof() {
		return core_model->dof_count;
	}

	Model::BodyID Model::get_body_id(const BodyIdentity &name) {
		const BodyID id = core_model->GetBodyId(name.c_str());

		if(std::numeric_limits<BodyID>::max() <= id) {
			throw std::runtime_error("No exist body ID");
		}

		return id;
	}

	Model::BodyIdentity Model::get_body_identity(const BodyID &id) {
		const BodyIdentity identity = core_model->GetBodyName(id);

		if(identity.empty()) {
			throw std::runtime_error("No exist body name");
		}

		return identity;
	}

	void Model::create_core_model() {
		if(core_model) {
			throw std::runtime_error("Already created core model from Kinematics::Model");
		}
		core_model = std::make_unique<RawModel>();
	}

	void Model::create_model_from_urdf(const std::string &filename) {
		create_core_model();
		auto is_read = RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), core_model.get(), false);
		if(!is_read) {
			throw std::runtime_error("Can not read URDF file: " + filename + " from Kinematics::Model");
		}
	}

	void Model::create_vectors_from_model_dof() {
		if(!core_model) {
			throw std::runtime_error("Can not access core model from Kinematics::Model");
		}
		q = std::make_unique<VectorN>(VectorN::Zero(core_model->dof_count));
		qd = std::make_unique<VectorN>(VectorN::Zero(core_model->dof_count));
		qdd = std::make_unique<VectorN>(VectorN::Zero(core_model->dof_count));
		tau = std::make_unique<VectorN>(VectorN::Zero(core_model->dof_count));
	}
}

