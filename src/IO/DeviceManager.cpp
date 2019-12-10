
/**
  *
  * @file DeviceManager.cpp
  * @author Naoki Takahashi
  *
  **/

#include "DeviceManager.hpp"

#include <Tools/ConfigFileOperator/JsonLoader.hpp>

#include "SerialDeviceSelector.hpp"
#include "Communicator/SerialControlSelector.hpp"

namespace IO {
	DeviceManager::DeviceManager(RobotStatus::InformationPtr &robo_info) {
		this->robo_info = robo_info;
		logger_ptr = std::make_unique<Tools::Log::Logger>();

		load_robot_config = std::make_unique<LoadConfig::RobotConfig>(
			"./",
			this->robo_info->get_config_filename<RobotStatus::Information::RobotType::Humanoid>()
		);

		update_config();
	}

	DeviceManager::DeviceManager(RobotStatus::InformationPtr &robo_info, Tools::Log::LoggerPtr &logger_ptr) {
		this->robo_info = robo_info;
		this->logger_ptr = logger_ptr;

		load_robot_config = std::make_unique<LoadConfig::RobotConfig>(
			"./",
			this->robo_info->get_config_filename<RobotStatus::Information::RobotType::Humanoid>(),
			this->logger_ptr
		);

		update_config();
	}

	DeviceManager::DeviceManager(const std::string &dir, const std::string &configname, RobotStatus::InformationPtr &robo_info) {
        this->robo_info = robo_info;
        logger_ptr = std::make_unique<Tools::Log::Logger>();

        load_robot_config = std::make_unique<LoadConfig::RobotConfig>(
            dir,
            configname
        );

        update_config();
    }

    DeviceManager::DeviceManager(const std::string &dir, const std::string &configname, RobotStatus::InformationPtr &robo_info, Tools::Log::LoggerPtr &logger_ptr) {
        this->robo_info = robo_info;
        this->logger_ptr = logger_ptr;

        load_robot_config = std::make_unique<LoadConfig::RobotConfig>(
            dir,
            configname,
            this->logger_ptr
        );

        update_config();
    }

	DeviceManager::~DeviceManager() {
		if(update_servo_angle_thread) {
			update_servo_angle_thread->join();
		}
	}

	void DeviceManager::spawn_device() {
		spawn_serial_controller();
		spawn_sensor_device();
		spawn_control_board_device();
		spawn_actuator_device();
	}

	void DeviceManager::update_config() {
		load_robot_config->update();
	}

	void DeviceManager::force_update_config() {
		load_robot_config->force_update();
	}

	void DeviceManager::launch_device() {
		launch_sensor_device();
		launch_actuator_with_control_board();
	}

	auto make_serial_control(const std::string &key) {
		auto selector = std::make_unique<Communicator::SerialControlSelector>();

		return selector->choice_shared_object(key);
	}

	void DeviceManager::spawn_serial_controller() {
		logger_ptr->message(Tools::Log::MessageLevels::trace, "Spawn serial controller");

		if(!load_robot_config->port_config_data) {
			throw std::runtime_error("Failed spawn for serial_controller from IO::DeviceManager");
		}
		for(auto &&[id, data] : load_robot_config->port_config_data->serial_control) {
			serial_control_map[id] = make_serial_control(data.name);

			serial_control_map[id]->port_name(data.port_name);
			serial_control_map[id]->baud_rate(data.baud_rate);
			serial_control_map[id]->timeout_ms(data.timeout_ms);
		}

		logger_ptr->message(Tools::Log::MessageLevels::trace, "Success spawn serial controller");
	}

	void DeviceManager::spawn_sensor_device() {
		spawn_imu_device();
	}

	auto make_control_board(RobotStatus::InformationPtr &r, const std::string &key) {
		auto selector = std::make_unique<SerialDeviceSelector<Device::ControlBoard::SerialControlBoard>>(r);

		return selector->choice_object(key);
	}

	void DeviceManager::spawn_control_board_device() {
		logger_ptr->message(Tools::Log::MessageLevels::trace, "Spawn control board");

		if(!load_robot_config->control_board_config_data) {
			throw std::runtime_error("Failed spawn for control board device from IO::DeviceManager");
		}
		for(auto &&[id, data] : load_robot_config->control_board_config_data->control_board) {
			if(!is_exist_serial_id(data.serial_id)) {
				throw std::runtime_error("Failed not exist serial id from IO::DeviceManager");
			}

			control_board.reset();
			control_board = make_control_board(robo_info, data.name);
			control_board->register_controller(serial_control_map[data.serial_id]);
		}

		logger_ptr->message(Tools::Log::MessageLevels::trace, "Success spawn control board");
	}

	void DeviceManager::spawn_actuator_device() {
		spawn_servo_motor_device();
	}

	auto make_imu(RobotStatus::InformationPtr &r, const std::string &key) {
		auto selector = std::make_unique<SerialDeviceSelector<Device::Sensor::IMU::InertialMeasurementUnit>>(r);

		return selector->choice_object(key);
	}

	void DeviceManager::spawn_imu_device() {
		logger_ptr->message(Tools::Log::MessageLevels::trace, "Spawn IMU device");

		if(!load_robot_config->imu_config_data) {
			throw std::runtime_error("Failed spawn for imu device from IO::DeviceManager");
		}
		if(load_robot_config->imu_config_data->name == "None") {
			logger_ptr->message(Tools::Log::MessageLevels::trace, "IMU device is none");
			return;
		}
		imu = make_imu(robo_info, load_robot_config->imu_config_data->name);

		if(!is_exist_serial_id(load_robot_config->imu_config_data->serial_id)) {
			throw std::runtime_error("Failed not exist serial id from IO::DeviceManager");
		}
		imu->register_controller(serial_control_map[load_robot_config->imu_config_data->serial_id]);

		if(load_robot_config->imu_config_data->accel) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::Accelerometers);
		}
		if(load_robot_config->imu_config_data->gyro) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::Gyroscopes);
		}
		if(load_robot_config->imu_config_data->compas) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::Magnetometers);
		}
		if(load_robot_config->imu_config_data->eulerangle) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::EulerAngles);
		}
		if(load_robot_config->imu_config_data->quaternion) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::Quaternions);
		}
		if(load_robot_config->imu_config_data->heading) {
			imu->enable(Device::Sensor::IMU::InertialMeasurementUnit::Streams::Heading);
		}

		logger_ptr->message(Tools::Log::MessageLevels::trace, "Success spawn IMU device");
	}

	auto make_serial_servo_motor(RobotStatus::InformationPtr &r, const std::string &key) {
		auto selector = std::make_unique<SerialDeviceSelector<Device::Actuator::ServoMotor::SerialServoMotor>>(r);

		return selector->choice_object(key);
	}

	void DeviceManager::spawn_servo_motor_device() {
		logger_ptr->message(Tools::Log::MessageLevels::trace, "Spawn servo motor device");

		if(!load_robot_config->actuator_config_data) {
			throw std::runtime_error("Failed spawn for servo motor device from IO::DeviceManager");
		}
		for(auto &&[id, data] : load_robot_config->actuator_config_data->serial_motor) {
			if(!is_exist_serial_id(data.serial_id)) {
				throw std::runtime_error("Failed not exist serial id from IO::DeviceManager");
			}
			if("None" != data.name) {
				servo_motor_map[id] = make_serial_servo_motor(robo_info, data.name);
				servo_motor_map[id]->id(id);
				servo_motor_map[id]->register_controller(serial_control_map[data.serial_id]);
			}
		}
		//robo_info->create_servo_data_space(servo_motor_map.size());

		logger_ptr->message(Tools::Log::MessageLevels::trace, "Success spawn servo motor device");
	}

	void DeviceManager::launch_sensor_device() {
		if(imu) {
			imu->async_launch();

			logger_ptr->message(Tools::Log::MessageLevels::trace, "Async launched IMU device");
		}
		logger_ptr->message(Tools::Log::MessageLevels::trace, "Async launched servo motor device");
	}

	void DeviceManager::launch_actuator_with_control_board() {
		if(servo_motor_map.empty()) {
			throw std::runtime_error("Failed launch actuator");
		}

		if(control_board && !servo_motor_map.empty()) {
			const auto device_id = control_board->id();
			const auto serial_control_id = load_robot_config->control_board_config_data->control_board[device_id].serial_id;

			serial_control_map[serial_control_id]->async_launch();

			control_board->enable_power(true);
		}
		else if(!servo_motor_map.empty()) {
			ID device_id = 1;
			{
				int i = 0;
				for(auto &&[id, ptr] : servo_motor_map) {
					if(0 == i) {
						device_id = id;
					}
					else if(device_id != id) {
						throw std::runtime_error("Failed not support serial_id from IO::DeviceManager");
					}
					i ++;
				}
			}
			const auto serial_control_id = load_robot_config->control_board_config_data->control_board.at(device_id).serial_id;

			serial_control_map[serial_control_id]->async_launch();
		}

		for(auto &&[id, ptr] : servo_motor_map) {
			ptr->enable_torque(true);
		}
		for(auto &&[id, ptr] : servo_motor_map) {
			ptr->write_gain(13, 1, 1);
		}

		update_servo_angle_thread = std::make_unique<std::thread>(
			[this]() {
				const auto device_id = servo_motor_map.begin()->first;
				const auto serial_control_id = load_robot_config->control_board_config_data->control_board.at(device_id).serial_id;

				while(1) {
					if(robo_info->write_servo_data) {
						for(auto &&[id, ptr] : servo_motor_map) {
							const auto joint_id = load_robot_config->actuator_config_data->serial_motor[id].joint_id; 
							ptr->write_angle(robo_info->write_servo_data->at(joint_id - 1).latest().value);
						}
					}

					serial_control_map[serial_control_id]->wait_for_send_packets();
				}
			}
		);

		logger_ptr->message(Tools::Log::MessageLevels::trace, "Async launched Control board device and servo motor device");
	}

	bool DeviceManager::is_exist_serial_id(const SerialID &serial_id) {
		return serial_control_map.find(serial_id) != serial_control_map.cend();
	}
}
