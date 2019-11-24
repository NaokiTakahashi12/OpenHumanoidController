
/**
  *
  * @file ConfigManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>
#include <vector>

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace Kinematics {
	class ConfigManager {
		public :
			using Ptr = std::unique_ptr<ConfigManager>;

			using JsonLoader = Tools::ConfigFileOperator::JsonLoader;

			ConfigManager(const std::string &dir, const std::string &filename);

			ConfigManager(const ConfigManager &) = delete;

			virtual ~ConfigManager();

			static Ptr make_ptr(const std::string &dir, const std::string &filename);

			const std::string filename();
			const std::string dir();

			template <typename T>
			T get_value(const std::string &key);

			template <typename T>
			std::vector<T> get_value_tree(const std::string &tree_key, const std::string &key);

			JsonLoader &raw();

		private :
			using JsonLoaderPtr = std::unique_ptr<JsonLoader>;

			const std::string config_dir,
				              config_filename;

			JsonLoaderPtr json_loader;

			void make_json_ptr();

	};
}

