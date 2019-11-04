
/**
  *
  * @file ConfigManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ConfigManager.hpp"

namespace FootStepPlanner {
	ConfigManager::ConfigManager(const std::string &config_file_name) {
		json_loader = std::make_unique<JsonLoader>(config_file_name);
	}

	ConfigManager::~ConfigManager() {
	}

	typename ConfigManager::Ptr ConfigManager::make_ptr(const std::string &config_file_name) {
		return std::make_unique<ConfigManager>(config_file_name);
	}

	std::string ConfigManager::file_name() {
		return json_loader->get_filename();
	}

	template <typename T>
	T ConfigManager::get_value(const std::string &value_identity) {
		return json_loader->get_parameter<T>(value_identity);
	}

	ConfigManager::JsonLoader &ConfigManager::raw() {
		return *(json_loader.get());
	}

	template int ConfigManager::get_value<int>(const std::string &);
	template float ConfigManager::get_value<float>(const std::string &);
	template double ConfigManager::get_value<double>(const std::string &);
	template std::string ConfigManager::get_value<std::string>(const std::string &);
}

