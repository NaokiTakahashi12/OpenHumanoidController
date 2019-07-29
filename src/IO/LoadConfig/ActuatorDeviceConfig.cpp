
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
		ActuatorDeviceConfig::ActuatorDeviceConfig(const std::string &config_file_name) {
			this->config_file_name = config_file_name;
			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}

		ActuatorDeviceConfig::ActuatorDeviceConfig(const std::string &config_file_name, Tools::Log::LoggerPtr &logger_ptr) {
			this->config_file_name = config_file_name;
			this->logger_ptr = logger_ptr;
		}

		void ActuatorDeviceConfig::update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update actuator device config now");
			if(!servomotor_config) {
				servomotor_config = std::make_unique<ServoMotorConfigData>();
			}
			update_serial_servo_config();
		}

		void ActuatorDeviceConfig::force_update() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Force update actuator device config now");
			servomotor_config = std::make_unique<ServoMotorConfigData>();
			update_serial_servo_config();
		}

		void ActuatorDeviceConfig::update_serial_servo_config() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Update config from " + config_file_name);
			Tools::ConfigFileOperator::JsonLoader mather_config_file(config_file_name);
			Tools::ConfigFileOperator::JsonLoader servo_config_file(mather_config_file.get_parameter<std::string>(servo_config_file_name));
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Loading servo motor config from " + servo_config_file.get_filename());
			Tools::ConfigFileOperator::JsonLoader servo_id_config_file(mather_config_file.get_parameter<std::string>(servo_id_config_file_name));
			logger_ptr->message(Tools::Log::MessageLevels::trace, "Loading servo motor ID config from " + servo_id_config_file.get_filename());

			servomotor_config->control_board_name = servo_config_file.get_parameter<std::string>(control_board_type_name);
			servomotor_config->path = servo_config_file.get_parameter<std::string>(servo_path_name);
			servomotor_config->servo_name = servo_config_file.get_parameter<std::string>(servo_type_name);

			for(auto &&ici : id_config_identities) {
				std::string load_identities = servomotor_id_head_name + ici;
				servomotor_config->id_list.push_back(servo_id_config_file.get_parameter<int>(load_identities));
				servomotor_config->id_map[ici] = servo_id_config_file.get_parameter<int>(load_identities);
			}

			print_out_status();
		}

		void ActuatorDeviceConfig::print_out_status() {
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Servo motor name is " + servomotor_config->servo_name);
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Control board name is " + servomotor_config->control_board_name);
			logger_ptr->message(Tools::Log::MessageLevels::trace, " Servo motor path is " + servomotor_config->path);
			for(auto &&[id_name, id] : servomotor_config->id_map) {
				logger_ptr->message(Tools::Log::MessageLevels::trace, " " + id_name + "," + std::to_string(id) + " from map");
			}
		}
	}
}

