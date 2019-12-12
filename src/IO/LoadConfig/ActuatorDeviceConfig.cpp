
/**
  *
  * @file ActuatorDeviceConfig.cpp
  * @author Naoki Takahashi
  *
  **/

#include "ActuatorDeviceConfig.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

namespace IO {
	namespace LoadConfig {
		ActuatorDeviceConfig::ActuatorDeviceConfig(const std::string &config_dir, const std::string &config_file_name) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		ActuatorDeviceConfig::ActuatorDeviceConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_dir = config_dir;
			this->config_file_name = config_file_name;

			this->logger_ptr = logger_ptr;
		}

		void ActuatorDeviceConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update actuator device config now");

			if(!config_data) {
				config_data = std::make_unique<ActuatorDeviceData>();
				load_config();
			}
		}

		void ActuatorDeviceConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Force update actuator device config now");

			if(config_data) {
				config_data.reset();
			}
			config_data = std::make_unique<ActuatorDeviceData>();

			load_config();
		}

		void ActuatorDeviceConfig::load_config() {
			Tools::ConfigFileOperator::JsonLoader config(config_dir + config_file_name);

			load_serial_servo_config(
				config.get_parameter<std::string>(serial_servo_motor_config_key)
			);

			print_serial_servo_from_logger();
		}

		void ActuatorDeviceConfig::load_serial_servo_config(const std::string &serial_config_filename) {
			Tools::ConfigFileOperator::JsonLoader config(config_dir + serial_config_filename);

			const auto dev_ids    = config.get_parameter_tree<int>(serial_servo_motor_tree_key, serial_servo_id_key);
			const auto serial_ids = config.get_parameter_tree<int>(serial_servo_motor_tree_key, serial_servo_serial_id_key);
			const auto dev_names  = config.get_parameter_tree<std::string>(serial_servo_motor_tree_key, serial_servo_name_key);
			const auto angle_ids  = config.get_parameter_tree<int>(angle_id_tree_key, angle_id_key);
			const auto angle_motor_ids  = config.get_parameter_tree<int>(angle_id_tree_key, angle_motor_id_key);

			if(dev_ids.size() != dev_names.size()) {
				throw std::runtime_error("Different size of Servo ID vs Servo name from IO::LoadConfig::ActuatorDeviceConfig");
			}
			else if(dev_ids.size() != serial_ids.size()) {
				throw std::runtime_error("Different size of Servo ID vs Servo serial id from IO::LoadConfig::ActuatorDeviceConfig");
			}
			else if(dev_ids.size() != angle_ids.size()) {
				throw std::runtime_error("Different size of Servo ID vs Angle ID from IO::LoadConfig::ActuatorDeviceConfig");
			}
			else if(dev_ids.size() != angle_motor_ids.size()) {
				throw std::runtime_error("Different size of Servo ID vs Angle motor ID from IO::LoadConfig::ActuatorDeviceConfig");
			}

			for(unsigned int i = 0; i < dev_ids.size(); i ++) {
				config_data->serial_motor[dev_ids.at(i)].name = dev_names.at(i);
				config_data->serial_motor[dev_ids.at(i)].serial_id = serial_ids.at(i);
				config_data->serial_motor[angle_motor_ids.at(i)].joint_id = angle_ids.at(i);
			}

			serial_servo_data_assertion();
		}

		void ActuatorDeviceConfig::serial_servo_data_assertion() {
			for(auto &&[id, data] : config_data->serial_motor) {
				if(data.name.empty()) {
					throw std::runtime_error("Failed empty serial servo name from IO::LoadConfig::ActuatorDeviceConfig");
				}
			}
		}

		void ActuatorDeviceConfig::print_serial_servo_from_logger() {
			for(auto &&[id, data] : config_data->serial_motor) {
				std::stringstream ss;

				ss << "Config_SerialServoMotor:";
				ss << " Name: " << data.name;
				ss << " SerialID: " << data.serial_id;
				ss << " ID: " << id;
				ss << " JointID: " << data.joint_id;

				logger_ptr->message(Tools::Log::MessageLevels::trace, ss.str());
			}
		}
	}
}

