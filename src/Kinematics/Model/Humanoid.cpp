
/**
  *
  * @file Humanoid.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Humanoid.hpp"

namespace Kinematics {
	namespace Model {
		Humanoid::Humanoid(const std::string &new_model_file_name) : RBDLBased(new_model_file_name) {
		}

		Humanoid::~Humanoid() {
		}

		typename Humanoid::Ptr Humanoid::make_ptr_from_urdf(const std::string &new_model_file_name) {
			return std::make_shared<Humanoid>(new_model_file_name);
		}
	}
}


