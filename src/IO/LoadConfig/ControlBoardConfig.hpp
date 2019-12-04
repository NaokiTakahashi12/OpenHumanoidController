
/**
  *
  * @file ControlBoardConfig.hpp
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "LoadConfigBase.hpp"

#include <Tools/Log/Logger.hpp>

#include "SerialPortConfig.hpp"

namespace IO {
	namespace LoadConfig {
		class ControlBoardConfig final : public LoadConfigBase {
			public :
				ControlBoardConfig(const std::string &config_dir, const std::string &config_file_name);
				ControlBoardConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &);

				ControlBoardConfig(const ControlBoardConfig &) = delete;

				struct ControlBoardData {
					using ControlBoardName = std::string;
					//! Map key
					using DeviceID = int;

					ControlBoardName name;
					SerialPortConfig::SerialControlData::SerialID serial_id;
				};

				struct ConfigData {
					using ControlBoardMap = std::unordered_map<ControlBoardData::DeviceID, ControlBoardData>;

					ControlBoardMap control_board;
				};

				std::unique_ptr<ConfigData> config_data;

				void update() override final;
				void force_update() override final;

			private :
				std::string config_dir,
							config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string serial_control_group_key = "Control Board.",
					  			  serial_control_board_name_key      = serial_control_group_key + "Name",
								  serial_control_board_id_key        = serial_control_group_key + "ID",
								  serial_control_board_serial_id_key = serial_control_group_key + "Serial ID";

				void load_config();

				void print_serial_control_board_from_logger();
		};
	}
}

