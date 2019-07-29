
/**
  *
  * @file RobotConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "RobotConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace IO {
	namespace LoadConfig {
		RobotConfig::RobotConfig(const std::string &config_file_name) {
			this->config_file_name = config_file_name;
			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		RobotConfig::RobotConfig(const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_file_name = config_file_name;
			this->logger_ptr = logger_ptr;
		}

		void RobotConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::info, "Update robot config now");
			Tools::ConfigFileOperator::JsonLoader config_file(config_file_name);

			if(!load_sensor_device_config) {
				load_sensor_device_config = std::make_unique<SensorDeviceConfig>(config_file.get_parameter<std::string>(sensor_device_config_identitiy), logger_ptr);
			}
			if(!load_actuator_device_config) {
				load_actuator_device_config = std::make_unique<ActuatorDeviceConfig>(config_file.get_parameter<std::string>(actuator_device_config_identity), logger_ptr);
			}

			load_sensor_device_config->update();
			load_actuator_device_config->update();

			move_data();
		}

		void RobotConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::info, "Update robot config now");
			Tools::ConfigFileOperator::JsonLoader config_file(config_file_name);

			load_sensor_device_config = std::make_unique<SensorDeviceConfig>(config_file.get_parameter<std::string>(sensor_device_config_identitiy), logger_ptr);
			load_actuator_device_config = std::make_unique<ActuatorDeviceConfig>(config_file.get_parameter<std::string>(actuator_device_config_identity), logger_ptr);

			load_sensor_device_config->force_update();
			load_actuator_device_config->force_update();

			move_data();
		}

		void RobotConfig::move_data() {
			if(load_sensor_device_config->imu_config) {
				imu_config_data = std::move(load_sensor_device_config->imu_config);
			}
			if(load_actuator_device_config->servomotor_config) {
				servomotor_config_data = std::move(load_actuator_device_config->servomotor_config);
			}
		}
	}
}

