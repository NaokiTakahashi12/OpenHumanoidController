
/**
  *
  * @file Dynamixel.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Dynamixel.hpp"

#include "../Protocols/DynamixelVersion1.hpp"

namespace IO {
	namespace Communicator {
		namespace  SerialController {
			Dynamixel::Dynamixel() : SerialControllerBase() {
				baud_rate(Protocols::DynamixelVersion1::default_baudrate);
			}

			Dynamixel::Dynamixel(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControllerBase(robot_status_information_ptr) {
			}

			Dynamixel::~Dynamixel() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			void Dynamixel::launch() {
				serial_flow_scheduler->register_parse(create_data_parser());
				serial_flow_scheduler->open(port_name());
				{
					serial_flow_scheduler->set_baudrate(baud_rate());
					serial_flow_scheduler->set_flow_control_none();
					serial_flow_scheduler->set_parity_none();
				}
				serial_flow_scheduler->launch();
				serial_flow_scheduler->run();
			}

			void Dynamixel::async_launch() {
				async_launch_thread = std::make_unique<Thread>(&Dynamixel::launch, this);
			}

			void Dynamixel::packet(const SendPacket &packet) {
				serial_flow_scheduler->set_send_packet(packet);
			}

			Dynamixel::DynamixelData Dynamixel::catch_packet(const ID &id) {
				return data_map[id];
			}

			Dynamixel::IDList Dynamixel::catch_packet_id() {
				IDList id_list;
				for(auto &&dm : data_map) {
					id_list.push_back(dm.first);
				}
				return id_list;
			}

			bool Dynamixel::is_exist(const ID &id) {
				return data_map.find(id) != data_map.end();
			}

			Dynamixel::ParseFunction Dynamixel::create_data_parser() {
				return [this](const ReadBuffer &read_buffer, const Length &length) {
					return packet_splitter(read_buffer, length);
				};
			}

			bool Dynamixel::packet_splitter(const ReadBuffer &read_buffer, const Length &length) {
				static unsigned int head_position;
				if(Protocols::DynamixelVersion1::is_broken_packet(read_buffer, head_position)) {
					head_position = 0;
					return false;
				}
				data_parser(read_buffer, head_position);
				head_position = Protocols::DynamixelVersion1::full_packet_size(read_buffer, head_position) + 1;
				if(head_position < length) {
					packet_splitter(read_buffer, length);
				}
				head_position = 0;
				return true;
			}

			void Dynamixel::data_parser(const ReadBuffer &read_buffer, const unsigned int &head_position) {
				const auto id = Protocols::DynamixelVersion1::packet_id(read_buffer, head_position);
				const auto data_length = Protocols::DynamixelVersion1::packet_length(read_buffer, head_position) - Protocols::DynamixelVersion1::number_of_error_size;

				data_map[id].error_code = Protocols::DynamixelVersion1::packet_error_code(read_buffer, head_position);
				data_map[id].parameters.resize(data_length);
				auto read_buffer_begin = read_buffer.begin() + head_position + Protocols::DynamixelVersion1::constant_head_byte_size() + 1;
				std::copy(
					read_buffer_begin,
					read_buffer_begin + data_length,
					data_map[id].parameters.begin()
				);
			}
		}
	}
}

