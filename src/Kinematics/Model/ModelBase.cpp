
/**
  *
  * @file ModelBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ModelBase.hpp"

namespace Kinematics {
	namespace Model {
		ModelBase::ModelBase(const std::string &new_model_file_name) {
			model_file_name = new_model_file_name;
			access_model_mutex = std::make_unique<std::mutex>();
		}

		ModelBase::~ModelBase() {
		}

		std::mutex &ModelBase::get_access_model_mutex() {
			return *access_model_mutex;
		}

		std::string ModelBase::model_file() {
			return model_file_name;
		}
	}
}

