
/**
  *
  * @file KondoB3M.hpp
  * @brief Kondo B3M communication protocol address
  * @authers Yasuo Hayashibara
  *			 Naoki Takahashi
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
			namespace KondoB3M {
				/**
				  *
				  * Send format
				  *	+--------+---------+--------+----+----------+-----+----------+-----+
				  *	| Size   | Command | Option | ID | S-Data_0 | ... | S-Data_n | Sum |
				  *	+--------+---------+--------+----+----------+-----+----------+-----+
				  * | 1 byte | ...
				  *
				  * Recieve format
				  *	+--------+---------+--------+----+----------+-----+----------+-----+
				  *	| Size   | Command | Status | ID | R-Data_0 | ... | R-Data_n | Sum |
				  *	+--------+---------+--------+----+----------+-----+----------+-----+
				  * | 1 byte | ...
				  *
				  **/

				using Byte = uint8_t;
				using Bytes = std::string;
				using ID = Byte;
				using Packet = Bytes;
				using ReadBuffer = Communicator::SerialFlowScheduler::ReadBuffer;
				using SyncWriteData = std::vector<std::string>;

				constexpr unsigned int default_baudrate = 1500000;

				constexpr int size_bytes = 2,
							  command_bytes = 1,
							  option_bytes = 1,
							  status_bytes = 1,
							  id_bytes = 1,
							  length_bytes = 1,
							  checksum_bytes = 1;

				constexpr Byte enable = 0x01,
						  		   disable = 0x00;
				
				constexpr bool illegal_id(const Byte &);

				constexpr Byte broadcast_id = 255;

				namespace Command {
					constexpr Byte
						load = 0x01,
						save = 0x02,
						read = 0x03,
						write = 0x04,
						reset = 0x05,
						position = 0x06;
				}

				namespace Option {
					constexpr Byte
						none = 0x00,
				    	system_status = 0x01,
					    motor_status = 0x02,
					    uart_status = 0x04,
					    command_status = 0x08,
						status_clear = 0x80;
				}
				
				namespace Error {
					constexpr Byte
						none = 0x00,
				    	system_status = 0x01,
					    motor_status = 0x02,
					    uart_status = 0x04,
					    command_status = 0x08;
				}

				Byte &create_checksum(const Packet &, const size_t &size = 0),
					 &create_checksum(const ReadBuffer &, const size_t &size = 0);

				Packet &create_read_packet(const Byte &id, const Byte &start_point, const Byte &read_size),
					   &create_write_packet(const Byte &id, const Byte &start_point, const Bytes &write_data),
					   &create_reset_packet(const Byte &id),
					   &create_position_packet(const Byte &id, const Byte &start_point, const Byte &position),
					   &create_position_packet(const Byte &id, const Byte &start_point, const Byte &positions);

				int constant_recieve_data_header_byte_size(),
					constant_recieve_data_tail_byte_size();

				uint8_t recieved_data_size(const Packet &),
					    recieved_data_size(const ReadBuffer &);

				int full_packet_size(const Packet &),
					full_packet_size(const ReadBuffer &);

				uint8_t packet_id(const Packet &),
						packet_id(const ReadBuffer &);

				uint8_t packet_error_code(const ReadBuffer &);

				bool is_broken_packet(const Packet &),
					 is_broken_packet(const ReadBuffer &);

				bool is_exist_header(const Packet &, const unsigned int &head_position = 0),
					 is_exist_header(const ReadBuffer &, const unsigned int &head_position = 0);
			}
		}
	}
}

