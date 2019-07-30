
/**
  *
  * @file KondoB3M.cpp
  * @auther Yasuo Hayashibara
  *
  **/

#include "KondoB3M.hpp"

#include <stdexcept>

namespace IO {
	namespace Communicator {
		namespace Protocols {
			namespace KondoB3M {
				constexpr bool illegal_id(const uint8_t &id) {
					if(id > 0xfd) {
						return true;
					}
					return false;
				}

				Byte &create_checksum(const Packet &packet, const size_t &size) {
					static Byte checksum;

					checksum = 0x0;
                    auto packet_size = (size == 0) ? packet.size() : size;
					for(auto itr = packet.begin(), end_itr = itr + packet_size; itr < end_itr; ++ itr) {
						checksum += *itr;
					}

					return checksum;
				}

				Byte &create_checksum(const ReadBuffer &packet, const size_t &size) {
					static Byte checksum;

					checksum = 0x0;
                    auto packet_size = (size == 0) ? packet.size() : size;
					for(auto itr = packet.begin(), end_itr = itr + packet_size; itr < end_itr; ++ itr) {
						checksum += *itr;
					}

					return checksum;
				}

				Packet &create_read_packet(const Byte &id, const Byte &start_address, const Byte &read_size) {
					constexpr uint8_t packet_length = 7;
					static Packet packet;

					packet = packet_length;
					packet += Command::read;
					packet += Option::none;
					packet += id;
					packet += start_address;
					packet += read_size;
					packet += create_checksum(packet);

					return packet;
				}

				Packet &create_write_packet(const Byte &id, const Byte &start_address, const Bytes &write_data) {
					constexpr uint8_t number_of_devices = 1;
					static Packet packet;
					static uint8_t packet_length;
                    packet_length = 7 + write_data.size();

					packet = packet_length;
					packet += Command::write;
					packet += Option::none;
					packet += id;
                    packet += write_data;
					packet += start_address;
                    packet += number_of_devices;
					packet += create_checksum(packet);

					return packet;
				}
				
				Packet &create_reset_packet(const Byte &id) {
					constexpr uint8_t packet_length = 6;
					constexpr uint8_t resset_immidiately = 0;
					static Packet packet;

					packet = packet_length;
					packet += Command::reset;
					packet += Option::none;
					packet += id;
					packet += resset_immidiately;
					packet += create_checksum(packet);

					return packet;
				}

				int constant_recieve_data_header_byte_size() {
					return size_bytes + command_bytes + status_bytes + id_bytes;
				}

				int constant_recieve_data_tail_byte_size() {
					return checksum_bytes;
				}

				uint8_t recieved_data_size(const Packet &packet) {
					return packet.size() - constant_recieve_data_header_byte_size() - constant_recieve_data_tail_byte_size();
				}

				uint8_t recieved_data_size(const ReadBuffer &read_buffer) {
					return read_buffer.size() - constant_recieve_data_header_byte_size() - constant_recieve_data_tail_byte_size();
				}

				int full_packet_size(const Packet &packet) {
                    return packet.size();
				}

				int full_packet_size(const ReadBuffer &read_buffer) {
					return read_buffer.size();
				}

				uint8_t packet_id(const Packet &packet) {
					return packet.at(size_bytes + command_bytes + status_bytes);
				}

				uint8_t packet_id(const ReadBuffer &read_buffer) {
					return read_buffer.at(size_bytes + command_bytes + status_bytes);
				}

				uint8_t packet_error_code(const ReadBuffer &read_buffer) {
					return read_buffer.at(size_bytes + command_bytes);
				}

				bool is_broken_packet(const Packet &packet) {
					const auto checksum = create_checksum(packet, packet.size() - 1);
					if(static_cast<Byte>(packet.at(packet.size() - 1)) != checksum) {
						return true;
					}
					return false;
				}

				bool is_broken_packet(const ReadBuffer &read_buffer) {
					const auto checksum = create_checksum(read_buffer, read_buffer.size() - 1);
					if(read_buffer.at(read_buffer.size() - 1) != checksum) {
						return true;
					}
					return false;
				}
			}
		}
	}
}
