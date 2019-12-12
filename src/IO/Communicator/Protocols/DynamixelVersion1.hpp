
/**
  *
  * @file DynamixelCommunicationProtocolV1.hpp
  * @brief Dynamixel communication protocol version 1.0 address
  * @auther Naoki Takahashi
  * @todo Create base class
  *
  **/

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		namespace Protocols {
			namespace DynamixelVersion1 {
				/**
				  *
				  * Send command packet
				  *	+--------+--------+----+--------+-------------+-------------+-----+-------------+----------+
				  *	| Header | Header | ID | Length | Instruction | Parameter_1 | ... | Parameter_n | Checksum |
				  *	+--------+--------+----+--------+-------------+-------------+-----+-------------+----------+
				  * | 1 byte | ...
				  *
				  * Catch status packet
				  *	+--------+--------+----+--------+-------+-------------+-----+-------------+----------+
				  *	| Header | Header | ID | Length | Error | Parameter_1 | ... | Parameter_n | Checksum |
				  *	+--------+--------+----+--------+-------+-------------+-----+-------------+----------+
				  * | 1 byte | ...
				  *
				  **/

				using Byte = uint8_t;
				using Bytes = std::string;
				using ID = Byte;
				using Packet = Bytes;
				using ReadBuffer = Communicator::SerialFlowScheduler::ReadBuffer;
				using SyncWriteData = std::vector<std::string>;

				constexpr unsigned int default_baudrate = 1000000;

				constexpr int number_of_header_size = 2,
							  number_of_id_size = 1,
							  number_of_length_size = 1,
							  number_of_error_size = 1,
							  number_of_checksum_size = 1;

				constexpr Byte enable = 0x01,
						  		   disable = 0x00;

				const Bytes header = {
					static_cast<char>(0xff),
					static_cast<char>(0xff)
				};
				
				constexpr bool illegal_id(const Byte &);

				constexpr Byte broadcast_id = 254;

				namespace Instructions {
					constexpr Byte
						ping = 0x01, 		// Parameter size = 0
				   		read = 0x02,			// Parameter size = 2
				    	write = 0x03,		// Parameter size >= 2
				    	reg_write = 0x04,	// Parameter size >= 2
				    	action = 0x05,		// Parameter size = 0
				    	reset = 0x06,		// Parameter size = 0
				    	sync_write = 0x83;	// Parameter size >= 4
				}

				namespace Errors {
					constexpr Byte
						none = 0x00,
				    	input_voltage = 0x01,
					    angle_limit = 0x02,
					    overheating = 0x04,
					    range = 0x08,
					    checksum = 0x10,
					    overload = 0x20,
				    	instruction = 0x40;
				}

				Byte &create_checksum(const Packet &),
						 &create_checksum(const Packet &, const unsigned int &top),
						 &create_checksum(const Packet &, const unsigned int &top, const size_t &size),
						 &create_checksum(const ReadBuffer &, const unsigned int &top, const size_t &size);

				Packet &create_ping_packet(const Byte &id),
						   &create_read_packet(const Byte &id, const Byte &start_point, const Byte &read_size),
						   &create_write_packet(const Byte &id, const Byte &start_point, const Byte &write_data),
						   &create_write_packet(const Byte &id, const Byte &start_point, const Bytes &write_datas),
						   &create_reg_write_packet(const Byte &id, const Byte &start_point, const Byte &write_data),
						   &create_reg_write_packet(const Byte &id, const Byte &start_point, const Bytes &write_datas),
						   &create_action_packet(const Byte &id),
						   &create_reset_packet(const Byte &id),
						   &create_sync_write(const Bytes &ids, const Byte &start_point, const SyncWriteData &write_datas); 

				int constant_head_byte_size(),
					constant_tail_byte_size();

				uint8_t packet_length(const Packet &, const unsigned int &head_position = 0),
					    packet_length(const ReadBuffer &, const unsigned int &head_position = 0);

				int full_packet_size(const Packet &, const unsigned int &head_position = 0),
					full_packet_size(const ReadBuffer &, const unsigned int &head_position = 0);

				uint8_t packet_id(const Packet &, const unsigned int &head_position = 0),
						packet_id(const ReadBuffer &, const unsigned int &head_position = 0);

				uint8_t packet_error_code(const ReadBuffer &, const unsigned int &head_position = 0);

				bool is_broken_packet(const Packet &, const unsigned int &head_position = 0),
					 is_broken_packet(const ReadBuffer &, const unsigned int &head_position = 0);

				bool is_exist_header(const Packet &, const unsigned int &head_position = 0),
					 is_exist_header(const ReadBuffer &, const unsigned int &head_position = 0);
			}
		}
	}
}

