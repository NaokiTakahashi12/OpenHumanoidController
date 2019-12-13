
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
				timeoutus = std::make_unique<TimeoutUs>();
				device_port_name = std::make_unique<std::string>();
				return_packet_map = std::make_unique<ReturnPacketMap>();
				data_access_mutex = std::make_unique<std::mutex>();

				baud_rate(0);
				timeout_us(1000);
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

			void SerialControllerBase::timeout_us(const TimeoutUs &new_timeout_us) {
				*timeoutus = new_timeout_us;
			}

			SerialControllerBase::TimeoutUs &SerialControllerBase::timeout_us() const {
				return *timeoutus;
			}

			void SerialControllerBase::launch() {
				throw std::runtime_error("Unoverride from IO::Communicator::SerialController::SerialControllerBase");
			}

			void SerialControllerBase::async_launch() {
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
//				printf("P");
#if 0
				printf("set_packet: ");
				for(int i = 0; i < send_packet[0]; i ++) {
					printf("%02X ", send_packet[i] & 0xFF);
				}
				printf("\r\n");
#endif
			}

			void SerialControllerBase::wait_for_send_packets() const {
				if(!serial_flow_scheduler) {
					return;
				}
				else if(serial_flow_scheduler->is_open()) {
					while(!serial_flow_scheduler->is_send_packet_empty()) {
						std::this_thread::yield();
						std::this_thread::sleep_for(std::chrono::microseconds(100));
					}
				}
			}

			SerialReturnPacket &SerialControllerBase::return_packet(const SerialReturnPacket::PacketID &packet_id) const {
				const auto lock = std::lock_guard<std::mutex>(*data_access_mutex);

				return (*return_packet_map)[packet_id];
			}

			bool SerialControllerBase::is_exist_return_packet(const SerialReturnPacket::PacketID &packet_id) const {
				const auto lock = std::lock_guard<std::mutex>(*data_access_mutex);

				return return_packet_map->find(packet_id) != return_packet_map->cend();
			}

			SerialControllerBase::ReturnPacketMap &SerialControllerBase::access_return_packet_map() {
				const auto lock = std::lock_guard<std::mutex>(*data_access_mutex);

				return *return_packet_map;
			}

			SerialFlowScheduler::ParseFunction SerialControllerBase::create_data_parser() {
				throw std::runtime_error("Unoverride from IO::Communicator::SerialController::SerialControllerBase");
			}
		}
	}
}

