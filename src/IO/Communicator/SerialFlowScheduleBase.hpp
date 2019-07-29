
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
				SerialFlowScheduleBase(boost::asio::io_service &);
				SerialFlowScheduleBase(boost::asio::serial_port &);

				virtual ~SerialFlowScheduleBase();

				static constexpr size_t maximum_read_buffer = 128;
				using Byte = uint8_t;
				using ReadBuffer = std::array<Byte, maximum_read_buffer>;
				using Length = std::size_t;
				using SinglePacket = std::string;
				using PacketList = std::deque<SinglePacket>;
				using ParseFunction = std::function<bool(const ReadBuffer &, const Length &)>;

				std::string get_error_message() const;

				void close();

				bool is_open();
				bool open(const std::string &portname);

				void launch();

				void run();

				void set_send_packet(const SinglePacket &);
				void register_parse(ParseFunction);

				void set_timeout_ms(const unsigned int &timeout_ms = 15),
					 set_read_end_sleep_ms(const unsigned int &read_end_sleep_ms),
					 set_write_end_sleep_ms(const unsigned int &write_end_sleep_ms),
					 set_baudrate(const unsigned int &baudrate = 115200),
					 set_character_size(const unsigned int &size = 8),
					 set_flow_control_none(),
					 set_stop_bits_is_one(),
					 set_parity_none(),
					 set_parity_even(),
					 set_parity_odd();

				bool is_send_packet_empty();

			protected :
				std::unique_ptr<std::mutex> mutex;

				PacketList send_packet_list;
				ReadBuffer read_buffer;

				ParseFunction parse_data_functor;

				virtual void read_timeout(),
							 read(),
							 write();

				virtual void initializer(),
							 initializer(boost::asio::io_service &),
							 initializer(boost::asio::serial_port &);

				virtual void default_settings();

				virtual void clean_read_buffer();

			private :
				unsigned int timeout_ms;
				std::chrono::milliseconds read_end_sleep_ms,
							 			  write_end_sleep_ms;

				std::unique_ptr<boost::asio::serial_port> port;
				std::unique_ptr<boost::asio::deadline_timer> timer;
				std::unique_ptr<boost::system::error_code> error_code;

				void timeout_callback(const boost::system::error_code &),
					 read_callback(const boost::system::error_code &, std::size_t),
					 write_callback(const boost::system::error_code &, std::size_t);

				void read_end(),
					 write_end();
		};
	}
}

