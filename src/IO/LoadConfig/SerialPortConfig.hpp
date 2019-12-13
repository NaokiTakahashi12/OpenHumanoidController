
/**
  *
  * @file SerialPortConfig.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "LoadConfigBase.hpp"

#include <unordered_map>

#include <Tools/Log/Logger.hpp>

namespace IO {
	namespace LoadConfig {
		class SerialPortConfig : public LoadConfigBase {
			public :
				SerialPortConfig(const std::string &config_dir, const std::string &config_file_name);
				SerialPortConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &);

				SerialPortConfig(const SerialPortConfig &) = delete;

				struct SerialControlData {
					using SerialControlName = std::string;
					using Port = std::string;
					using BaudRate = unsigned int;
					using TimeOutUs = int;
					//! Map key
					using SerialID = int;

					SerialControlName name;
					Port port_name;
					BaudRate baud_rate;
					TimeOutUs timeout_us;
				};

				struct SerialPortData {
					using SerialControlMap = std::unordered_map<SerialControlData::SerialID, SerialControlData>;

					SerialControlMap serial_control;
				};

				std::unique_ptr<SerialPortData> config_data;

				void update() override final;
				void force_update() override final;

			private :
				std::string config_dir,
							config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string serial_control_tree_key = "Serial control",
								  serial_controller_key    = "Serial controller",
								  serial_port_key          = "Port",
								  serial_baud_rate_key     = "Baud rate",
								  serial_timeout_key       = "Timeout[us]",
								  serial_controller_id_key = "Serial ID";

				void load_config();

				void serial_control_assertion();

				void print_serial_control_from_logger();
		};
	}
}

