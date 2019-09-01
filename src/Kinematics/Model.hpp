
/**
  *
  * @file Model.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>
#include <string>

#include <rbdl/rbdl.h>

namespace Kinematics {
	class Model {
		public :
			using RawModel = RigidBodyDynamics::Model;

			using VectorNElement = double;
			using VectorN = RigidBodyDynamics::Math::VectorNd;

			using BodyID = unsigned int;
			using BodyIdentity = std::string;

			using DofSize = unsigned int;

			Model();
			Model(const std::string &urdf_filename);

			void setup_from_urdf(const std::string &filename);

			bool is_load_model();

			const RawModel raw();
			RawModel &access();
			
			VectorN get_q(),
					get_qd(),
					get_qdd(),
					get_tau();

			VectorN &access_q(),
					&access_qd(),
					&access_qdd(),
					&access_tau();

			DofSize number_of_dof();

			BodyID get_body_id(const BodyIdentity &name);
			BodyIdentity get_body_identity(const BodyID &body_id);

		private :
			using VectorNPtr = std::unique_ptr<VectorN>;
			using RawModelPtr = std::unique_ptr<RawModel>;

			RawModelPtr core_model;

			VectorNPtr q, qd, qdd, tau;

			void create_core_model();

			void create_model_from_urdf(const std::string &filename);

			void create_vectors_from_model_dof();

	};
	using ModelPtr = std::shared_ptr<Model>;

}

