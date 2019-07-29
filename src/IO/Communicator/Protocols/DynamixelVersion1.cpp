
/**
  *
  * @file DynamixelCommunicationProtocolV1.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "DynamixelVersion1.hpp"

#include <stdexcept>

namespace IO {
	namespace Communicator {
		namespace Protocols {
			namespace DynamixelVersion1 {
				constexpr bool illegal_id(const uint8_t &id) {
					if(id > 0xfd) {
						return true;
					}
					return false;
				}

				Byte &create_checksum(const Packet &packet) {
					static Byte checksum;

					checksum = 0x0;
					for(auto itr = packet.begin(); itr < packet.end(); ++ itr) {
						checksum += *itr;
					}

					checksum = ~checksum;

					return checksum;
				}

				Byte &create_checksum(const Packet &packet, const unsigned int &top) {
					static Byte checksum;

					checksum = 0x0;
					for(auto itr = packet.begin() + top; itr < packet.end(); ++ itr) {
						checksum += *itr;
					}

					checksum = ~checksum;

					return checksum;
				}

				Byte &create_checksum(const Packet &packet, const unsigned int &top, const size_t &size) {
					static Byte checksum;

					checksum = 0x0;
					for(auto itr = packet.begin() + top, end_itr = itr + size; itr < end_itr; ++ itr) {
						checksum += *itr;
					}

					checksum = ~checksum;

					return checksum;
				}

				Byte &create_checksum(const ReadBuffer &packet, const unsigned int &top, const size_t &size) {
					static Byte checksum;

					checksum = 0x0;
					for(auto itr = packet.begin() + top, end_itr = itr + size; itr < end_itr; ++ itr) {
						checksum += *itr;
					}

					checksum = ~checksum;

					return checksum;
				}

				Packet &create_ping_packet(const Byte &id) {
					constexpr uint8_t packet_length = 2;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::ping;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}

				Packet &create_read_packet(const Byte &id, const Byte &start_point, const Byte &read_size) {
					constexpr uint8_t packet_length = 4;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::read;
					packet += start_point;
					packet += read_size;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}

				Packet &create_write_packet(const Byte &id, const Byte &start_point, const Byte &write_data) {
					constexpr uint8_t packet_length = 4;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::write;
					packet += start_point;
					packet += write_data;
					packet += create_checksum(packet, 2);

					return packet;
				}

				Packet &create_write_packet(const Byte &id, const Byte &start_point, const Bytes &write_datas) {
					static Packet packet;
					static uint8_t packet_length;
					packet_length = 2 + write_datas.size() + 1;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::write;
					packet += start_point;
					packet += write_datas;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}
				
				Packet &create_reg_write_packet(const Byte &id, const Byte &start_point, const Byte &write_data) {
					constexpr uint8_t packet_length = 4;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::reg_write;
					packet += start_point;
					packet += write_data;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}

				Packet &create_reg_write_packet(const Byte &id, const Byte &start_point, const Bytes &write_datas) {
					static Packet packet;
					static uint8_t packet_length;
					packet_length = 2 + write_datas.size() + 1;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::reg_write;
					packet += start_point;
					packet += write_datas;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}

				Packet &create_action_packet(const Byte &id) {
					constexpr uint8_t packet_length = 2;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::action;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}
				
				Packet &create_reset_packet(const Byte &id) {
					constexpr uint8_t packet_length = 2;
					static Packet packet;

					packet = header;
					packet += id;
					packet += packet_length;
					packet += Instructions::reset;
					packet += create_checksum(packet, number_of_header_size + number_of_id_size - 1);

					return packet;
				}

				Packet &create_sync_write(const Bytes &ids, const Byte &start_point, const SyncWriteData &write_datas) {
					static Packet packet;
					static uint8_t packet_length;
					static uint8_t write_byte_size;

					if(ids.size() != write_datas.size()) {
						throw std::runtime_error("Error not match from Dynamixel communication v1 sync-write");
					}
					packet_length = 4;
					write_byte_size = write_datas.front().size() + 1;

					if(write_byte_size <= 0) {
						throw std::runtime_error("Sync-write data >= 1");
					}
					for(auto &&wd : write_datas) {
						if(wd.size() != write_byte_size) {
							packet_length += wd.size() + 1;
						}
						else {
							throw std::runtime_error("Illegal write data from Dynamixel communication v1 sync-write");
						}
					}
					packet = header;
					packet += broadcast_id;
					packet += packet_length;
					packet += Instructions::sync_write;
					packet += start_point;
					packet += write_byte_size;

					auto write_itr = write_datas.begin();
					for(auto &&id : ids) {
						packet += id;
						packet += *write_itr;
					}
					packet += create_checksum(packet, 2);

					return packet;
				}

				int constant_head_byte_size() {
					return number_of_header_size + number_of_id_size + number_of_length_size;
				}

				int constant_tail_byte_size() {
					return number_of_checksum_size;
				}

				uint8_t packet_length(const Packet &packet, const unsigned int &head_position) {
					return packet.at(head_position + number_of_header_size + number_of_id_size);
				}

				uint8_t packet_length(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					return read_buffer.at(head_position + number_of_header_size + number_of_id_size);
				}

				int full_packet_size(const Packet &packet, const unsigned int &head_position) {
					return packet_length(packet, head_position) + constant_head_byte_size();
				}

				int full_packet_size(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					return packet_length(read_buffer, head_position) + constant_head_byte_size();
				}

				uint8_t packet_id(const Packet &packet, const unsigned int &head_position) {
					return packet.at(head_position + number_of_header_size);
				}

				uint8_t packet_id(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					return read_buffer.at(head_position + number_of_header_size);
				}

				uint8_t packet_error_code(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					return read_buffer.at(head_position + constant_head_byte_size());
				}

				bool is_broken_packet(const Packet &packet, const unsigned int &head_position) {
					if(!is_exist_header(packet, head_position)) {
						return true;
					}
					const auto packet_size = packet_length(packet, head_position);
					const auto checksum_byte = create_checksum(packet, head_position + number_of_header_size, packet_size + number_of_checksum_size);
					if(static_cast<Byte>(packet.at(head_position + number_of_header_size + number_of_id_size + packet_size)) != checksum_byte) {
						return true;
					}
					return false;
				}

				bool is_broken_packet(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					if(!is_exist_header(read_buffer, head_position)) {
						return true;
					}
					const auto packet_size = packet_length(read_buffer, head_position);
					const auto checksum_byte = create_checksum(read_buffer, head_position + number_of_header_size, packet_size + number_of_checksum_size);
					if(read_buffer.at(head_position + number_of_header_size + number_of_id_size + packet_size) != checksum_byte) {
						return true;
					}
					return false;
				}

				bool is_exist_header(const Packet &packet, const unsigned int &head_position) {
					for(auto i = head_position; i < head_position + number_of_header_size; ++ i) {
						if(header.at(i - head_position) != packet.at(i)) {
							return false;
						}
					}
					return true;
				}

				bool is_exist_header(const ReadBuffer &read_buffer, const unsigned int &head_position) {
					for(auto i = head_position; i < head_position + number_of_header_size; ++ i) {
						if(static_cast<Byte>(header.at(i - head_position)) != read_buffer.at(i)) {
							return false;
						}
					}
					return true;
				}
			}
		}
	}
}
