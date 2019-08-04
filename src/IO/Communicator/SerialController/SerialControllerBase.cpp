
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
				baudrate = std::make_unique<BaudRate>();
				device_port_name = std::make_unique<std::string>();
				baud_rate(0);
				serial_flow_scheduler = std::make_unique<Communicator::SerialFlowScheduler>();
			}

			SerialControllerBase::~SerialControllerBase() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			void SerialControllerBase::port_name(const std::string &new_device_port_name) {
				*device_port_name = new_device_port_name;
			}

			std::string &SerialControllerBase::port_name() const {
				return *device_port_name;
			}

			void SerialControllerBase::baud_rate(const BaudRate &new_baudrate) {
				*baudrate = new_baudrate;
			}

			SerialControllerBase::BaudRate &SerialControllerBase::baud_rate() const {
				return *baudrate;
			}

			void SerialControllerBase::launch() {
				throw std::runtime_error("Unoverride from IO::Communicator::SerialController::SerialControllerBase");
			}

			void SerialControllerBase::async_launch() {
				throw std::runtime_error("Unoverride from IO::Communicator::SerialController::SerialControllerBase");
			}

			SerialFlowScheduler::ParseFunction SerialControllerBase::create_data_parser() {
				throw std::runtime_error("Unoverride from IO::Communicator::SerialController::SerialControllerBase");
			}

			void SerialControllerBase::register_parse(ParseFunction data_parse_function) {
				if(!serial_flow_scheduler) {
					throw std::runtime_error("Can not register data parse function from IO::Communicator::SerialController::SerialControllerBase");
				}
				serial_flow_scheduler->register_parse(data_parse_function);
			}

			void SerialControllerBase::set_packet(const SendPacket &send_packet) {
				if(!serial_flow_scheduler) {
					throw std::runtime_error("Can not set packet from IO::Communicator::SerialController::SerialControllerBase");
				}
				serial_flow_scheduler->set_send_packet(send_packet);
			}

			void SerialControllerBase::wait_for_send_packets() const {
				if(!serial_flow_scheduler) {
					return;
				}
				else if(serial_flow_scheduler->is_open()) {
					if(!serial_flow_scheduler->is_send_packet_empty()) {
						std::this_thread::yield();
						wait_for_send_packets();
					}
				}
			}
		}
	}
}

