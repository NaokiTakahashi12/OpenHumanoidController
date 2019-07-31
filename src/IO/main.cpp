
/**
  *
  * @file main.cpp
  * @brief IO module test
  *
  **/

#include <exception>
#include <bitset>
#include <iostream>

#include <Tools/CommandLineArgumentReader.hpp>
#include <Tools/Log/Logger.hpp>

#include <RobotStatus/Information.hpp>

#include "Communicator/SerialFlowScheduler.hpp"
#include "Communicator/TCP.hpp"

#include "Device/Sensor/IMU/VMU931.hpp"
#include "Device/Sensor/IMU/InertialMeasurementUnit.hpp"

#include "Device/ControlBoard/CM730.hpp"
#include "Device/ControlBoard/SerialControlBoard.hpp"

#include "Device/Actuator/ServoMotor/MX28.hpp"
#include "Device/Actuator/ServoMotor/SerialServoMotor.hpp"

#include "DeviceManager.hpp"

#include "Communicator/Protocols/DynamixelVersion1.hpp"
#include "Communicator/SerialController/Dynamixel.hpp"

#include "Robot.hpp"

void serial_base_test(Tools::Log::LoggerPtr, const std::string &);

int main(int argc, char **argv) {
	auto robo_info = std::make_shared<RobotStatus::Information>(argc, argv);
	auto logger = robo_info->logger;

	try {
		logger->start_loger_thread();
		logger->message(Tools::Log::MessageLevels::warning, "This is IO test code");
		logger->message(Tools::Log::MessageLevels::info, "IO module test start");
		logger->message(Tools::Log::MessageLevels::debug, "Starttime access count of " + std::to_string(logger.use_count()));
		logger->message(Tools::Log::MessageLevels::info, "");

		Tools::CommandLineArgumentReader clar(argc, argv);
        std::string imu_port_name = clar.get("-imu");
		std::string servo_port_name = clar.get("-servo");

		auto robot = std::make_unique<IO::Robot>(robo_info);

		if(!servo_port_name.empty()) {
			logger->message(Tools::Log::MessageLevels::info, "Servo control device name is " + servo_port_name);
			auto serial_controller = std::make_shared<IO::Communicator::SerialController::Dynamixel>();
			auto control_board = std::make_unique<IO::Device::ControlBoard::CM730>(robo_info);
			auto serial_servo_motors = std::vector<IO::Device::Actuator::ServoMotor::MX28>();

			control_board->register_controller(serial_controller);
			for(auto i = 1; i <= 20; ++ i) {
				serial_servo_motors.push_back(IO::Device::Actuator::ServoMotor::MX28(i, robo_info));
				serial_servo_motors.back().register_controller(serial_controller);
			}

			serial_controller->port_name(servo_port_name);
			serial_controller->baud_rate(1000000);
			serial_controller->async_launch();

			control_board->enable_power(true);
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
            for(auto &&ssm : serial_servo_motors) {
				ssm.enable_torque(true);
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            for(auto &&ssm : serial_servo_motors) {
				ssm.write_gain(1, 0, 0);
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
			while(true) {
				serial_controller->wait_for_send_packets();
			}
		}
		else {
			logger->message(Tools::Log::MessageLevels::info, "Unset Servo port name");
		}

		if(!imu_port_name.empty()) {
			logger->message(Tools::Log::MessageLevels::info, "IMU device name is " + imu_port_name);
			auto imu_device = std::make_unique<IO::Device::Sensor::IMU::VMU931>(robo_info);

			imu_device->enable_all();
			logger->message(Tools::Log::MessageLevels::info, "Enable IMU functions");
			imu_device->port_name(imu_port_name);
			logger->message(Tools::Log::MessageLevels::info, "Set IMU port name");
			imu_device->async_launch();
			logger->message(Tools::Log::MessageLevels::info, "IMU launch");

			while(true) {
				RobotStatus::TimeSeriesData<Tools::Math::Vector3<float>> accel, euler, gyro;
				if(robo_info->accelerometers_data) {
					const auto current_accel = robo_info->accelerometers_data->latest();
					if(current_accel != accel) {
						accel = current_accel;
						std::stringstream ss;
						ss << "acc: " << accel.value.transpose() << "\t" << accel.timestamp << "\t";
						logger->message(Tools::Log::MessageLevels::debug, ss.str());
					}
				}
				if(robo_info->eulerangles_data) {
					const auto current_euler = robo_info->eulerangles_data->latest();
					if(current_euler != euler) {
						euler = current_euler;
						std::stringstream ss;
						ss << "ang: " << euler.value.transpose() << "\t" << euler.timestamp << "\t";
						logger->message(Tools::Log::MessageLevels::debug, ss.str());
					}
				}
				if(robo_info->gyroscopes_data) {
					const auto current_gyro = robo_info->gyroscopes_data->latest();
					if(current_gyro != gyro) {
						gyro = current_gyro;
						std::stringstream ss;
						ss << "gyr: " << gyro.value.transpose() << "\t" << gyro.timestamp << "\t";
						logger->message(Tools::Log::MessageLevels::debug, ss.str());
					}
				}
			}
		}
		else {
			logger->message(Tools::Log::MessageLevels::info, "Unset IMU port name");
		}

		logger->message(Tools::Log::MessageLevels::info, "");
		logger->message(Tools::Log::MessageLevels::debug, "Dynamixel pakcet debug");

		std::vector<IO::Communicator::Protocols::DynamixelVersion1::Packet> debug_packets;
		std::vector<IO::Communicator::Protocols::DynamixelVersion1::ReadBuffer> debug_buffers;
		IO::Communicator::Protocols::DynamixelVersion1::Bytes debug_write_bytes;
		IO::Communicator::Protocols::DynamixelVersion1::SyncWriteData debug_sync_write_datas;

		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_ping_packet(0x01));
		debug_write_bytes = static_cast<char>(0x00);
		debug_write_bytes += static_cast<char>(0x02);
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_write_packet(0x01, 0x1e, debug_write_bytes));
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_read_packet(0x01, 0x2b, 0x01));
		debug_write_bytes = static_cast<char>(0xc8);
		debug_write_bytes += static_cast<char>(0x00);
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_reg_write_packet(0x01, 0x1e, debug_write_bytes));
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_action_packet(0x01));
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_reset_packet(0x01));
		debug_write_bytes = static_cast<char>(0x20);
		debug_write_bytes += static_cast<char>(0x02);
		debug_write_bytes += static_cast<char>(0x60);
		debug_write_bytes += static_cast<char>(0x03);
		debug_sync_write_datas.push_back(debug_write_bytes);
		debug_write_bytes = static_cast<char>(0x01);
		debug_packets.push_back(IO::Communicator::Protocols::DynamixelVersion1::create_sync_write(debug_write_bytes, 0x1e, debug_sync_write_datas));

		for(size_t j = 0; j < debug_packets.size(); ++j) {
			debug_buffers.push_back(IO::Communicator::Protocols::DynamixelVersion1::ReadBuffer());
			for(auto i = 0; i < IO::Communicator::Protocols::DynamixelVersion1::full_packet_size(debug_packets.at(j)); ++ i) {
				debug_buffers.at(j).at(i) = static_cast<uint8_t>(debug_packets.at(j).at(i));
			}
		}

		for(auto &&dp : debug_packets) {
			if(IO::Communicator::Protocols::DynamixelVersion1::is_broken_packet(dp)) {
				std::stringstream ss;
				for(auto &&p : dp) {
					ss << std::bitset<8>(p) << "\n";
				}
				logger->message(Tools::Log::MessageLevels::warning, "Broken packet");
				logger->message(Tools::Log::MessageLevels::warning, ss.str());
				break;
			}
			std::stringstream ss;
			ss << static_cast<int>(IO::Communicator::Protocols::DynamixelVersion1::packet_id(dp));
			logger->message(Tools::Log::MessageLevels::debug, "Success packet ID:" + ss.str());
		}
		for(auto &&db : debug_buffers) {
			if(IO::Communicator::Protocols::DynamixelVersion1::is_broken_packet(db)) {
				std::stringstream ss;
				for(auto &&b : db) {
					ss << std::bitset<8>(b) << "\n";
				}
				logger->message(Tools::Log::MessageLevels::warning, "Broken packet");
				logger->message(Tools::Log::MessageLevels::warning, ss.str());
				break;
			}
			std::stringstream ss;
			ss << static_cast<int>(IO::Communicator::Protocols::DynamixelVersion1::packet_id(db));
			logger->message(Tools::Log::MessageLevels::debug, "Success packet ID:" + ss.str());
		}

		logger->message(Tools::Log::MessageLevels::info, "");

		logger->message(Tools::Log::MessageLevels::info, "Load RobotConfig debug");
		robo_info->set_config_filename<RobotStatus::Information::RobotType::Humanoid>("robot.conf.json");
		robo_info->set_config_filename<RobotStatus::Information::DeviceType::Sensor>("sensor.conf.json");
		robo_info->set_config_filename<RobotStatus::Information::DeviceType::Actuator>("actuator.conf.json");

		auto device_manager = std::make_unique<IO::DeviceManager>(robo_info, logger);
		device_manager->update_config();

		logger->message(Tools::Log::MessageLevels::info, "");
	}
	catch(const std::exception &error) {
		logger->message(Tools::Log::MessageLevels::fatal, "Throwing error from main try");
		std::cerr << error.what() << std::endl;
	}

	logger->message(Tools::Log::MessageLevels::info, "");
	logger->message(Tools::Log::MessageLevels::debug, "Endtime access count of " + std::to_string(logger.use_count()));
	logger->message(Tools::Log::MessageLevels::info, "IO module test end");
	logger->close_loger_thread();

	return 0;
}

void communicator_for_tcp() {
		std::cout << "Starting communicator" << std::endl;

		IO::Communicator::TCP tcp(5000);

		std::cout << "Build client or server" << std::endl;
		std::cout << tcp.client("127.0.0.1").message() << std::endl;
		//std::cout << tcp.server_v4().message() << std::endl;

		std::cout << "Read or Write" << std::endl;
		std::cout << tcp.write("I'm client").message() << std::endl;
		//std::cout << tcp.read() << std::endl;

		std::cout << "End communicator" << std::endl;
}

void serial_base_test(Tools::Log::LoggerPtr logger, const std::string &port_name) {
	if(!port_name.empty()) {
		logger->message(Tools::Log::MessageLevels::info, "IMU device path is " + port_name);
		auto serial_ptr = std::make_shared<IO::Communicator::SerialFlowScheduler>();

		if(serial_ptr->open(port_name)) {
			logger->message(Tools::Log::MessageLevels::info, "Successful serial port connection");
		}
		else {
			throw std::runtime_error("Can not open serial port " + port_name);
		}

		IO::Communicator::SerialFlowScheduler::SinglePacket send_packet;
		send_packet = "vara";
		send_packet += "varg";
		serial_ptr->set_send_packet(send_packet);

		serial_ptr->register_parse(
				[&logger](const IO::Communicator::SerialFlowScheduler::ReadBuffer &read_buffer, const std::size_t &bytes) {
					if(*(read_buffer.begin() + 2) == 'a') {
						std::stringstream ss;
						for(auto itr = read_buffer.begin(), end_itr = read_buffer.begin() + bytes; itr < end_itr; ++itr) {
							ss << std::showbase << std::hex << static_cast<int>(*itr) << std::noshowbase;
							ss << ",";
						}
						ss << static_cast<int>(bytes);
						logger->message(Tools::Log::MessageLevels::debug, ss.str());
					}
					return true;
				}
		);

		serial_ptr->launch();
	}
}

