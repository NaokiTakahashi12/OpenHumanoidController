
/**
  *
  * @file ConfigManager.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <memory>

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace FootStepPlanner {
	class ConfigManager {
		public :
			using Ptr = std::unique_ptr<ConfigManager>;
			using JsonLoader = Tools::ConfigFileOperator::JsonLoader;

			ConfigManager(const std::string &config_file_name);
			virtual ~ConfigManager();

			static Ptr make_ptr(const std::string &config_file_name);

			std::string file_name();

			template <typename T>
			T get_value(const std::string &value_identity);

			JsonLoader &raw();

		private :
			using JsonLoaderPtr = std::unique_ptr<JsonLoader>;

			JsonLoaderPtr json_loader;

	};
}

