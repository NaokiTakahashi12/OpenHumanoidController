
/**
  *
  * @file SerialControllerBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "SerialControllerBase.hpp"

#include <stdexcept>

namespace IO {
	namespace Communicator {
		namespace SerialController {
			SerialControllerBase::SerialControllerBase() {
				baudrate = 0;
			}

			SerialControllerBase::SerialControllerBase(RobotStatus::InformationPtr &robot_status_information_ptr) {
				SerialControllerBase();
				robo_info = robot_status_information_ptr;
				io_service = std::make_unique<boost::asio::io_service>();
				serial_flow_scheduler = std::make_unique<Communicator::SerialFlowScheduler>(*io_service);
			}

			SerialControllerBase::~SerialControllerBase() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			void SerialControllerBase::port_name(const std::string &device_port_name) {
				this->device_port_name = device_port_name;
			}

			std::string SerialControllerBase::port_name() const {
				return device_port_name;
			}

			void SerialControllerBase::baud_rate(const BaudRate &baudrate) {
				this->baudrate = baudrate;
			}

			SerialControllerBase::BaudRate SerialControllerBase::baud_rate() const {
				return baudrate;
			}

			void SerialControllerBase::launch() {
				throw std::runtime_error("Un override");
			}

			void SerialControllerBase::async_launch() {
				throw std::runtime_error("Un override");
			}

			SerialFlowScheduler::ParseFunction SerialControllerBase::create_data_parser() {
				throw std::runtime_error("Un override");
			}
		}
	}
}

