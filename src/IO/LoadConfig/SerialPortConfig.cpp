
/**
  *
  * @file SerialPortConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialPortConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace IO {
	namespace LoadConfig {
		SerialPortConfig::SerialPortConfig(const std::string &config_dir, const std::string &config_file_name) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		SerialPortConfig::SerialPortConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			this->logger_ptr = logger_ptr;
		}

		void SerialPortConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update serial port config now");

			if(!config_data) {
				config_data = std::make_unique<SerialPortData>();
				load_config();
			}
		}

		void SerialPortConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Force update serial port config now");

			if(config_data) {
				config_data.reset();
			}
			config_data = std::make_unique<SerialPortData>();

			load_config();
		}

		void SerialPortConfig::load_config() {
			Tools::ConfigFileOperator::JsonLoader config(config_dir + config_file_name);

			const auto names = config.get_parameter_tree<std::string>(serial_control_tree_key, serial_controller_key);
			const auto ports = config.get_parameter_tree<std::string>(serial_control_tree_key, serial_port_key);
			const auto baud_rates = config.get_parameter_tree<int>(serial_control_tree_key, serial_baud_rate_key);
			const auto timeouts   = config.get_parameter_tree<int>(serial_control_tree_key, serial_timeout_key);
			const auto ids        = config.get_parameter_tree<int>(serial_control_tree_key, serial_controller_id_key);

			if(names.size() != ports.size()) {
				throw std::runtime_error("Different size of SerialControl name vs SerialContorl port from IO::LoadConfig::SerialPortConfig");
			}
			else if(names.size() != baud_rates.size()) {
				throw std::runtime_error("Different size of SerialControl name vs SerialContorl baud rate from IO::LoadConfig::SerialPortConfig");
			}
			else if(names.size() != timeouts.size()) {
				throw std::runtime_error("Different size of SerialControl name vs SerialContorl timeout from IO::LoadConfig::SerialPortConfig");
			}
			else if(names.size() != ids.size()) {
				throw std::runtime_error("Different size of SerialControl name vs SerialContorl ID from IO::LoadConfig::SerialPortConfig");
			}

			for(unsigned int i = 0; i < names.size(); i ++) {
				config_data->serial_control[ids.at(i)].name = names.at(i);
				config_data->serial_control[ids.at(i)].port_name = ports.at(i);
				config_data->serial_control[ids.at(i)].baud_rate = baud_rates.at(i);
				config_data->serial_control[ids.at(i)].timeout_ms = timeouts.at(i);
			}

			serial_control_assertion();

			print_serial_control_from_logger();
		}

		void SerialPortConfig::serial_control_assertion() {
			for(auto &&[id, data] : config_data->serial_control) {
				if(data.name.empty() || data.port_name.empty()) {
					throw std::runtime_error("Failed empty serial_control name or port_name from IO::LoadConfig::SerialPortConfig");
				}
			}
		}

		void SerialPortConfig::print_serial_control_from_logger() {
			for(auto &&[id, data] : config_data->serial_control) {
				std::stringstream ss;

				ss << "Config_SerialControl:";
				ss << " Name: " << data.name;
				ss << " SerialID: " << id;
				ss << " Port: " << data.port_name;
				ss << " BaudRate: " << data.baud_rate;
				ss << " TimeoutMs: " << data.timeout_ms;

				logger_ptr->message(Tools::Log::MessageLevels::trace, ss.str());
			}
		}
	}
}

