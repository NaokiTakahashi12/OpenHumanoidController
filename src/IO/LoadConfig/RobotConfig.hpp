
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

#include "SensorDeviceConfig.hpp"
#include "ActuatorDeviceConfig.hpp"

namespace IO {
	namespace LoadConfig {
		class RobotConfig final : public LoadConfigBase {
			public :
				RobotConfig(const std::string &config_filename);
				RobotConfig(const std::string &config_filename, Tools::Log::LoggerPtr &);

				RobotConfig(const RobotConfig &) = delete;

				void update() override final;
				void force_update() override final;

				std::unique_ptr<SensorDeviceConfig::IMUConfigData> imu_config_data;
				std::unique_ptr<ActuatorDeviceConfig::ServoMotorConfigData> servomotor_config_data;

			private :
				std::string config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string sensor_device_config_identitiy = "Sensor config file",
					  			  actuator_device_config_identity = "Actuator config file";

				std::unique_ptr<SensorDeviceConfig> load_sensor_device_config;
				std::unique_ptr<ActuatorDeviceConfig> load_actuator_device_config;

				void move_data();
		};
	}
}

