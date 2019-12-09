
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
		RobotConfig::RobotConfig(const std::string &config_dir, const std::string &config_file_name) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		RobotConfig::RobotConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			this->logger_ptr = logger_ptr;
		}

		void RobotConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::info, "Update robot config now");
			Tools::ConfigFileOperator::JsonLoader config_file(make_config_file_path());

			if(!load_serial_port_config) {
				load_serial_port_config = std::make_unique<SerialPortConfig>(
					config_dir,
					config_file.get_parameter<std::string>(serial_port_config_key),
					logger_ptr
				);
			}
			if(!load_sensor_device_config) {
				load_sensor_device_config = std::make_unique<SensorDeviceConfig>(
					config_dir,
					config_file.get_parameter<std::string>(sensor_device_config_key),
					logger_ptr
				);
			}
			if(!load_actuator_device_config) {
				load_actuator_device_config = std::make_unique<ActuatorDeviceConfig>(
					config_dir,
					config_file.get_parameter<std::string>(actuator_device_config_key),
					logger_ptr
				);
			}
			if(!load_control_board_device_config) {
				load_control_board_device_config = std::make_unique<ControlBoardConfig>(
					config_dir,
					config_file.get_parameter<std::string>(control_device_config_key),
					logger_ptr
				);
			}

			load_serial_port_config->update();
			load_sensor_device_config->update();
			load_actuator_device_config->update();
			load_control_board_device_config->update();

			move_data();
		}

		void RobotConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::info, "Update robot config now");
			Tools::ConfigFileOperator::JsonLoader config_file(make_config_file_path());

			load_serial_port_config = std::make_unique<SerialPortConfig>(
				config_dir,
				config_file.get_parameter<std::string>(serial_port_config_key),
				logger_ptr
			);
			load_sensor_device_config = std::make_unique<SensorDeviceConfig>(
				config_dir,
				config_file.get_parameter<std::string>(sensor_device_config_key),
				logger_ptr
			);
			load_actuator_device_config = std::make_unique<ActuatorDeviceConfig>(
				config_dir,
				config_file.get_parameter<std::string>(actuator_device_config_key),
				logger_ptr
			);
			load_control_board_device_config = std::make_unique<ControlBoardConfig>(
				config_dir,
				config_file.get_parameter<std::string>(control_device_config_key),
				logger_ptr
			);

			load_serial_port_config->force_update();
			load_sensor_device_config->force_update();
			load_actuator_device_config->force_update();
			load_control_board_device_config->force_update();

			move_data();
		}

		void RobotConfig::move_data() {
			if(load_serial_port_config->config_data) {
				port_config_data = std::move(load_serial_port_config->config_data);
			}
			if(load_sensor_device_config->imu_config) {
				imu_config_data = std::move(load_sensor_device_config->imu_config);
			}
			if(load_actuator_device_config->config_data) {
				actuator_config_data = std::move(load_actuator_device_config->config_data);
			}
			if(load_control_board_device_config->config_data) {
				control_board_config_data = std::move(load_control_board_device_config->config_data);
			}
		}

		std::string RobotConfig::make_config_file_path() {
			if(config_dir.empty() || config_file_name.empty()) {
				throw std::runtime_error("Failed config empty from IO::LoadConfig::RobotConfig");
			}

			return config_dir + config_file_name;
		}
	}
}

