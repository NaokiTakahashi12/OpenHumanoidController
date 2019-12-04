
/**
  *
  * @file Kondo.cpp
  * @author Naoki Takahashi
  *
  **/

#include "Kondo.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			Kondo::Kondo() : SerialControllerBase() {
				constexpr auto default_baud_rate = 1000000;
				baud_rate(default_baud_rate);
			}

			Kondo::~Kondo() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			std::string Kondo::get_key() {
				return "Kondo";
			}

			void Kondo::launch() {
				serial_flow_scheduler->open(port_name());
				{
					serial_flow_scheduler->set_baudrate(baud_rate());
					serial_flow_scheduler->set_flow_control_none();
					serial_flow_scheduler->set_parity_none();
				}
				serial_flow_scheduler->launch();
			}

			void Kondo::async_launch() {
				async_launch_thread = std::make_unique<Thread>(&Kondo::launch, this);
			}

			Kondo::ParseFunction Kondo::create_data_parser() {
				return [this](const ReadBuffer &read_buffer, const Length &length) {
					return packet_splitter(read_buffer, length);
				};
			}

			bool Kondo::packet_splitter(const ReadBuffer &read_buffer, const Length &length) {
				static Length head_position;
				const auto return_packet_length = static_cast<std::size_t>(read_buffer.at(head_position));

				//! Check of container size
				if(SerialFlowScheduler::maximum_read_buffer <= return_packet_length) {
					head_position = 0;
					return false;
				}
				else if(0 == return_packet_length || length <= return_packet_length) {
					head_position = 0;
					return false;
				}

				//! Check sum
				SerialFlowScheduler::Byte sum_check = 0;
				for(unsigned int i = 0; i < return_packet_length - 1; i ++) {
					sum_check = read_buffer.at(head_position + i);
				}
				if(sum_check != read_buffer.at(head_position + return_packet_length)) {
					head_position = 0;
					return false;
				}

				const auto contents_size = return_packet_length - 5;

				SerialReturnPacket ret_pack;
				ret_pack.status = read_buffer.at(head_position + 2);
				ret_pack.id = read_buffer.at(head_position + 3);
				ret_pack.contents.resize(contents_size);

				const auto contents_begin = read_buffer.cbegin() + head_position + 4;
				std::copy(contents_begin, contents_begin + contents_size, ret_pack.contents.begin());

				access_return_packet_map()[ret_pack.id] = ret_pack;
				head_position += return_packet_length;

				if(head_position < length) {
					packet_splitter(read_buffer, length);
				}
				head_position = 0;

				return true;
			}
		}
	}
}

