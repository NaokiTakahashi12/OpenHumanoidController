
/**
  *
  * @file ActuatorDeviceConfig.hpp
  * @brief Load actuator setting class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "LoadConfigBase.hpp"

#include <unordered_map>

#include <Tools/Log/Logger.hpp>

namespace IO {
	namespace LoadConfig {
		class ActuatorDeviceConfig : public LoadConfigBase {
			public :
				ActuatorDeviceConfig(const std::string &config_file_name);
				ActuatorDeviceConfig(const std::string &config_file_name, Tools::Log::LoggerPtr &);

				ActuatorDeviceConfig(const ActuatorDeviceConfig &) = delete;

				struct ServoMotorConfigData {
					using IDIdentityMap = std::unordered_map<std::string, int>;
					std::vector<int> id_list;
					IDIdentityMap id_map;
					std::string path,
								control_board_name,
								servo_name;
				};

				std::unique_ptr<ServoMotorConfigData> servomotor_config;

				void update() override final;
				void force_update() override final;

			private :
				std::string config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string servo_config_file_name = "Servo motor config file",
								  servo_id_config_file_name = "Servo motor id config file",
								  servo_type_name = "Servo motor",
								  control_board_type_name = "Control board",
								  servo_path_name = "Servo motor device path";

				const std::string servomotor_id_head_name = "Servo motor ID.";

				const std::vector<std::string> id_config_identities = {
					"Head yaw",
					"Head pitch",
					"Left shoulder pitch",
					"Left shoulder roll",
					"Left elbow",
					"Right shoulder pitch",
					"Right shoulder roll",
					"Right elbow",
					"Left hip yaw",
					"Left hip roll",
					"Left hip pitch",
					"Left knee",
					"Left ankle pitch",
					"Left ankle roll",
					"Right hip yaw",
					"Right hip roll",
					"Right hip pitch",
					"Right knee",
					"Right ankle pitch",
					"Right ankle roll",
				};

				void update_serial_servo_config();

				void print_out_status();
		};
	}
}

