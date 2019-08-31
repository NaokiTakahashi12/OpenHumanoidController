
/**
  *
  * @file Model.hpp
  * @author Naoki Takahashi
  *
  **/

#include <memory>
#include <string>

#include <rbdl/rbdl.h>

namespace Kinematics {
	class Model {
		private :
			using RawModel = RigidBodyDynamics::Model;
			using RawModelPtr = std::unique_ptr<RawModel>;

			RawModelPtr core_model;

			void create_core_model();

		public :
			Model();

			void create_model_from_urdf(const std::string &filename);

			const RawModel get_raw();
	};
}

