
/**
  *
  * @file ConfigManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ConfigManager.hpp"

namespace Kinematics {
	ConfigManager::ConfigManager(const std::string &dir, const std::string &filename) : config_dir(dir), config_filename(filename) {
		make_json_ptr();
	}

	ConfigManager::~ConfigManager() {
	}

	ConfigManager::Ptr ConfigManager::make_ptr(const std::string &dir, const std::string &filename) {
		return std::make_unique<ConfigManager>(dir, filename);
	}

	const std::string ConfigManager::filename() {
		return config_filename;
	}

	const std::string ConfigManager::dir() {
		return config_dir;
	}

	template <typename T>
	T ConfigManager::get_value(const std::string &key) {
		return json_loader->get_parameter<T>(key);
	}

	template <typename T>
	std::vector<T> ConfigManager::get_value_tree(const std::string &tree_key, const std::string &key) {
		return json_loader->get_parameter_tree<T>(tree_key, key);
	}

	ConfigManager::JsonLoader &ConfigManager::raw() {
		return *json_loader;
	}

	void ConfigManager::make_json_ptr() {
		json_loader = std::make_unique<JsonLoader>(config_dir + config_filename);
	}

	template int ConfigManager::get_value<int>(const std::string &);
	template float ConfigManager::get_value<float>(const std::string &);
	template std::string ConfigManager::get_value<std::string>(const std::string &);

	template std::vector<int> ConfigManager::get_value_tree<int>(const std::string &, const std::string &);
	template std::vector<float> ConfigManager::get_value_tree<float>(const std::string &, const std::string &);
	template std::vector<std::string> ConfigManager::get_value_tree<std::string>(const std::string &, const std::string &);
}

