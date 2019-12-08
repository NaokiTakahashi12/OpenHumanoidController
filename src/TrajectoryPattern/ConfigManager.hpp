
/**
  *
  * @file ConfigManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <string>

namespace TrajectoryPattern {
	class ConfigManager {
		public :
			ConfigManager(const std::string &dir, const std::string &filename);
			~ConfigManager();

			template <typename T>
			T get_value(const std::string &key);

		private :
			std::string dir,
						filename;
	};
}

