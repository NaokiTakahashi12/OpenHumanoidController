
/**
  *
  * @file RobotConfig.hpp
  * @brief Load robot setting class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "LoadConfigBase.hpp"

#include <Tools/Log/Logger.hpp>

#include "SerialPortConfig.hpp"
#include "SensorDeviceConfig.hpp"
#include "ActuatorDeviceConfig.hpp"
#include "ControlBoardConfig.hpp"

namespace IO {
	namespace LoadConfig {
		class RobotConfig final : public LoadConfigBase {
			public :
				RobotConfig(const std::string &config_dir, const std::string &config_filename);
				RobotConfig(const std::string &config_dir, const std::string &config_filename, Tools::Log::LoggerPtr &);

				RobotConfig(const RobotConfig &) = delete;

				void update() override final;
				void force_update() override final;

				std::unique_ptr<SerialPortConfig::SerialPortData> port_config_data;
				std::unique_ptr<SensorDeviceConfig::IMUConfigData> imu_config_data;
				std::unique_ptr<ActuatorDeviceConfig::ActuatorDeviceData> actuator_config_data;
				std::unique_ptr<ControlBoardConfig::ConfigData> control_board_config_data;

			private :
				std::string config_dir,
							config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string serial_port_group_key      = "IO.",
					  			  serial_device_group_key    = serial_port_group_key   + "Devices.",
					  			  serial_port_config_key     = serial_port_group_key   + "Serial port config file",
					  			  sensor_device_config_key   = serial_device_group_key + "Sensor config file",
					  			  actuator_device_config_key = serial_device_group_key + "Actuator config file",
					  			  control_device_config_key  = serial_device_group_key + "Control board config file";

				std::unique_ptr<SerialPortConfig> load_serial_port_config;
				std::unique_ptr<SensorDeviceConfig> load_sensor_device_config;
				std::unique_ptr<ActuatorDeviceConfig> load_actuator_device_config;
				std::unique_ptr<ControlBoardConfig> load_control_board_device_config;

				void move_data();

				std::string make_config_file_path();
		};
	}
}

