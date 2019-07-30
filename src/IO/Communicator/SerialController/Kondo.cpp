
/**
  *
  * @file Kondo.cpp
  * @author Yasuo Hayashibara
  *
  **/

#include "Kondo.hpp"

#include "../Protocols/KondoB3M.hpp"

namespace IO {
	namespace Communicator {
		namespace  SerialController {
			Kondo::Kondo() : SerialControllerBase() {
				baud_rate(Protocols::KondoB3M::default_baudrate);
			}

			Kondo::Kondo(RobotStatus::InformationPtr &robot_status_information_ptr) : SerialControllerBase(robot_status_information_ptr) {
			}

			Kondo::~Kondo() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			void Kondo::launch() {
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

			void Kondo::async_launch() {
				async_launch_thread = std::make_unique<Thread>(&Kondo::launch, this);
			}

			void Kondo::packet(const SendPacket &packet) {
				serial_flow_scheduler->set_send_packet(packet);
			}

			Kondo::KondoData Kondo::catch_packet(const ID &id) {
				return data_map[id];
			}

			Kondo::IDList Kondo::catch_packet_id() {
				IDList id_list;
				for(auto &&dm : data_map) {
					id_list.push_back(dm.first);
				}
				return id_list;
			}

			bool Kondo::is_exist(const ID &id) {
				return data_map.find(id) != data_map.end();
			}

			Kondo::ParseFunction Kondo::create_data_parser() {
				return [this](const ReadBuffer &read_buffer, const Length &length) {
					return data_parser(read_buffer, length);
				};
			}

			bool Kondo::data_parser(const ReadBuffer &read_buffer, const Length &length) {
				const auto id = Protocols::KondoB3M::packet_id(read_buffer);
				const auto data_length = Protocols::KondoB3M::recieved_data_size(read_buffer);

				data_map[id].error_code = Protocols::KondoB3M::packet_error_code(read_buffer);
				data_map[id].parameters.resize(data_length);
				auto read_buffer_begin = read_buffer.begin() + Protocols::KondoB3M::constant_recieve_data_header_byte_size();
				std::copy(
					read_buffer_begin,
					read_buffer_begin + data_length,
					data_map[id].parameters.begin()
				);
			}
		}
	}
}
