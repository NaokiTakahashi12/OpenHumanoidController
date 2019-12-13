
/**
  *
  * @file SerialFlowScheduleBase.hpp
  * @brief Serial communication management base class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include <deque>
#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/serial_port.hpp>

namespace IO {
	namespace Communicator {
		class SerialFlowScheduleBase {
			public :
				SerialFlowScheduleBase();

				virtual ~SerialFlowScheduleBase();

				static constexpr std::size_t maximum_read_buffer = 256;
				using Byte = uint8_t;
				using BaudRate = unsigned int;
				using ReadBuffer = std::array<Byte, maximum_read_buffer>;
				using Length = std::size_t;
				using SinglePacket = std::string;
				using PacketList = std::deque<SinglePacket>;
				using ParseFunction = std::function<bool(const ReadBuffer &, const Length &)>;

				std::string get_error_message() const;

				void close();

				bool is_open() const;
				bool open(const std::string &portname);

				void launch();

				void set_send_packet(const SinglePacket &);
				void register_parse(ParseFunction);

				SerialFlowScheduleBase &set_timeout_us(const unsigned int &timeout_us = 1000),
									   &set_read_end_sleep_us(const unsigned int &read_end_sleep_us),
									   &set_write_end_sleep_us(const unsigned int &write_end_sleep_us),
					 				   &set_baudrate(const BaudRate &baudrate = 115200),
									   &set_character_size(const unsigned int &size = 8),
									   &set_flow_control_none(),
									   &set_stop_bits_is_one(),
									   &set_parity_none(),
									   &set_parity_even(),
									   &set_parity_odd();

				bool is_send_packet_empty() const;

			protected :
				std::unique_ptr<std::mutex> packet_mutex;

				PacketList send_packet_list;
				ReadBuffer read_buffer;

				ParseFunction parse_data_functor;

				virtual void read_timeout(),
							 read(),
							 write();

				virtual void default_settings();

				virtual void clean_read_buffer();

			private :
				unsigned int timeout_us;
				std::chrono::microseconds read_end_sleep_us,
										  write_end_sleep_us;

				std::unique_ptr<boost::asio::io_service> io_service;
				std::unique_ptr<boost::asio::serial_port> port;
				std::unique_ptr<boost::asio::deadline_timer> timer;
				std::unique_ptr<boost::system::error_code> error_code;

				void timeout_callback(const boost::system::error_code &),
					 read_callback(const boost::system::error_code &, Length),
					 write_callback(const boost::system::error_code &, Length);

				void read_end(),
					 write_end();
		};
	}
}

