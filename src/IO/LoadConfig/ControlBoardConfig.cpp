
/**
  *
  * @file ControlBoardConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ControlBoardConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace IO {
	namespace LoadConfig {
		ControlBoardConfig::ControlBoardConfig(const std::string &config_dir, const std::string &config_file_name) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			this->logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		ControlBoardConfig::ControlBoardConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			this->logger_ptr = logger_ptr;
		}

		void ControlBoardConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update control board device config now");

			if(!config_data) {
				config_data = std::make_unique<ConfigData>();

				load_config();
			}
		}

		void ControlBoardConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Force update control board device config now");

			if(config_data) {
				config_data.reset();
			}
			config_data = std::make_unique<ConfigData>();

			load_config();
		}

		void ControlBoardConfig::load_config() {
			Tools::ConfigFileOperator::JsonLoader config(config_dir + config_file_name);

			auto id = config.get_parameter<int>(serial_control_board_id_key);
			config_data->control_board[id].serial_id = config.get_parameter<int>(serial_control_board_serial_id_key);
			config_data->control_board[id].name = config.get_parameter<std::string>(serial_control_board_name_key);

			print_serial_control_board_from_logger();
		}

		void ControlBoardConfig::print_serial_control_board_from_logger() {
			for(auto &&[id, data] : config_data->control_board) {
				std::stringstream ss;

				ss << "Config_ControlBoard:";
				ss << " Name: " << data.name;
				ss << " SerialID: " << data.serial_id;
				ss << " ID: " << id;

				logger_ptr->message(Tools::Log::MessageLevels::trace, ss.str());
			}
		}
	}
}

