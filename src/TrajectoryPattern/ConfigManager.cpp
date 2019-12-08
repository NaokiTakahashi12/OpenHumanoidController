
/**
  *
  * @file ConfigManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ConfigManager.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace TrajectoryPattern {
	ConfigManager::ConfigManager(const std::string &dir, const std::string &filename) {
		this->dir = dir;
		this->filename = filename;

		if(dir.empty()) {
			throw std::invalid_argument("Failed dir name empty from TrajectroyPattern::ConfigManager");
		}
		if(filename.empty()) {
			throw std::invalid_argument("Failed filename name empty from TrajectroyPattern::ConfigManager");
		}
	}
	
	ConfigManager::~ConfigManager() {
	}

	template <typename T>
	T ConfigManager::get_value(const std::string &key) {
		Tools::ConfigFileOperator::JsonLoader json(dir + filename);

		T value;
		value = json.get_parameter<int>(key);

		return value;
	}

	template bool ConfigManager::get_value<bool>(const std::string &);
	template int ConfigManager::get_value<int>(const std::string &);
	template float ConfigManager::get_value<float>(const std::string &);
	template std::string ConfigManager::get_value<std::string>(const std::string &);
}

