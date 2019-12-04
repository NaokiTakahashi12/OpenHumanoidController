
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

#include "SerialPortConfig.hpp"

namespace IO {
	namespace LoadConfig {
		class ActuatorDeviceConfig : public LoadConfigBase {
			public :
				ActuatorDeviceConfig(const std::string &config_dir, const std::string &config_file_name);
				ActuatorDeviceConfig(const std::string &config_dir, const std::string &config_file_name, Tools::Log::LoggerPtr &);

				ActuatorDeviceConfig(const ActuatorDeviceConfig &) = delete;

				struct SerialServoMotorData {
					using JointID = int;
					using DeviceName = std::string;
					//! Map key
					using DeviceID = int;

					JointID joint_id;
					DeviceName name;

					SerialPortConfig::SerialControlData::SerialID serial_id;
				};

				struct ActuatorDeviceData {
					using SerialServoMotorMap = std::unordered_map<SerialServoMotorData::DeviceID, SerialServoMotorData>;

					SerialServoMotorMap serial_motor;
				};

				std::unique_ptr<ActuatorDeviceData> config_data;

				void update() override final;
				void force_update() override final;

			private :
				std::string config_dir;
				std::string config_file_name;

				Tools::Log::LoggerPtr logger_ptr;

				const std::string serial_servo_motor_config_key = "Serial servo motor config";

				const std::string serial_servo_motor_tree_key = "Serial Servo Motor",
					  			  serial_servo_id_key         = "ID",
								  serial_servo_name_key       = "Motor type",
								  serial_servo_serial_id_key  = "Serial ID";

				const std::string angle_id_tree_key  = "Angle ID",
					  			  angle_id_key       = "ID",
								  angle_motor_id_key = "Motor ID";


				void load_config();
				void load_serial_servo_config(const std::string &);

				void serial_servo_data_assertion();
				void check_full_config_data();

				void print_serial_servo_from_logger();

		};
	}
}

