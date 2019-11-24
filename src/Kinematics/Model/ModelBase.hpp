
/**
  *
  * @file ModelBase.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <string>
#include <mutex>

namespace Kinematics {
	namespace Model {
		class ModelBase {
			public :
				ModelBase(const std::string &new_model_file_name);
				virtual ~ModelBase();

				virtual void initialize() = 0;

				std::string model_file();

			protected :
				std::mutex &get_access_model_mutex();

				virtual void load_model_from_file(const std::string &load_model_file) = 0;

			private :
				std::string model_file_name;

				std::unique_ptr<std::mutex> access_model_mutex;
		};
	}
}

