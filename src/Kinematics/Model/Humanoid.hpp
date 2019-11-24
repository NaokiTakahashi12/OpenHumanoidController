
/**
  *
  * @file Humanoid.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "RBDLBased.hpp"

namespace Kinematics {
	namespace Model {
		class Humanoid : public RBDLBased {
			public :
				using Ptr = std::shared_ptr<Humanoid>;

				Humanoid(const std::string &new_model_file_name);
				virtual ~Humanoid();

				static Ptr make_ptr_from_urdf(const std::string &new_model_file_name);
		};
	}
}

