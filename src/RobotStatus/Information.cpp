
/**
  *
  * @file Information.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Information.hpp"

namespace RobotStatus {
	Information::Information() {
		robot_type = RobotType::Null;
	}

	Information::Information(const int &argc, char **argv) {
		Information();
		register_logger(argc, argv);
	}

	template <>
	void Information::set_config_filename<Information::RobotType::Humanoid>(const ConfigName &humanoid_config_file_name) {
		robot_configname = std::make_unique<ConfigName>(humanoid_config_file_name);
		robot_type = Information::RobotType::Humanoid;
	}

	template <>
	void Information::set_config_filename<Information::RobotType::Cart>(const ConfigName &cart_config_file_name) {
		robot_configname = std::make_unique<ConfigName>(cart_config_file_name);
		robot_type = Information::RobotType::Cart;
	}

	template <>
	Information::ConfigName Information::get_config_filename<Information::RobotType::Humanoid>() const {
		if(robot_type == Information::RobotType::Humanoid) {
			return *robot_configname;
		}
		return NULL;
	}

	template <>
	Information::ConfigName Information::get_config_filename<Information::RobotType::Cart>() const {
		if(robot_type == Information::RobotType::Cart) {
			return *robot_configname;
		}
		return NULL;
	}

	template <>
	void Information::set_config_filename<Information::DeviceType::Sensor>(const ConfigName &imu_config_file_name) {
		sensor_configname = std::make_unique<ConfigName>(imu_config_file_name);
	}

	template <>
	void Information::set_config_filename<Information::DeviceType::Actuator>(const ConfigName &servo_config_file_name) {
		actuator_configname = std::make_unique<ConfigName>(servo_config_file_name);
	}

	template <>
	Information::ConfigName Information::get_config_filename<Information::DeviceType::Sensor>() const {
		if(sensor_configname) {
			return *sensor_configname;
		}
		return NULL;
	}

	template <>
	Information::ConfigName Information::get_config_filename<Information::DeviceType::Actuator>() const {
		if(actuator_configname) {
			return *actuator_configname;
		}
		return NULL;
	}

	template <>
	bool Information::empty_config_filename<Information::DeviceType::Sensor>() const {
		return nullptr != sensor_configname.get();
	}

	template <>
	bool Information::empty_config_filename<Information::DeviceType::Actuator>() const {
		return nullptr != actuator_configname.get();
	}

	void Information::register_logger(int argc, char **argv) {
		if(!logger) {
			logger = std::make_shared<Tools::Log::Logger>(argc, argv);
		}
	}

	void Information::create_imu_data_space(const int &size) {
		create_accel_data_space(size);
		create_gyro_data_space(size);
		create_magnet_data_space(size);
	}

	void Information::create_accel_data_space(const int &size) {
		if(!accelerometers_data) {
			accelerometers_data = std::make_unique<AccelerometersDataType>(size);
		}
	}

	void Information::create_gyro_data_space(const int &size) {
		if(!gyroscopes_data) {
			gyroscopes_data = std::make_unique<GyroscopesDataType>(size);
		}
	}

	void Information::create_magnet_data_space(const int &size) {
		if(!magnetometers_data) {
			magnetometers_data = std::make_unique<MagnetometersDataType>(size);
		}
	}

	void Information::create_euler_data_space(const int &size) {
		if(!eulerangles_data) {
			eulerangles_data = std::make_unique<EulerAnglesDataType>(size);
		}
	}

	void Information::create_quat_data_space(const int &size) {
		if(!quaternions_data) {
			quaternions_data = std::make_unique<QuaternionsDataType>(size);
		}
	}

	void Information::create_head_data_space(const int &size) {
		if(!heading_data) {
			heading_data = std::make_unique<HeadingDataType>(size);
		}
	}

	void Information::create_servo_data_space(const int &size) {
		if(!read_servo_data && !write_servo_data) {
			read_servo_data = std::make_unique<ServoDataType>(size);
			write_servo_data = std::make_unique<ServoDataType>(size);
		}
	}
}

