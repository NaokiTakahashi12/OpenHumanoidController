
/**
  *
  * @file SerialFlowScheduleBase.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "SerialFlowScheduleBase.hpp"

#include <stdexcept>
#include <thread>

#include <boost/asio/write.hpp>

namespace IO {
	namespace Communicator {
		SerialFlowScheduleBase::SerialFlowScheduleBase(boost::asio::io_service &io_service) {
			initializer(io_service);
		}

		SerialFlowScheduleBase::SerialFlowScheduleBase(boost::asio::serial_port &serial_port) {
			initializer(serial_port);
		}

		SerialFlowScheduleBase::~SerialFlowScheduleBase() {
			close();
		}

		std::string SerialFlowScheduleBase::get_error_message() const {
			return error_code->message();
		}

		void SerialFlowScheduleBase::close() {
			if(port && is_open()) {
				port->cancel();
				port->close();
				port.reset();
			}
		}

		bool SerialFlowScheduleBase::is_open() {
			return port->is_open();
		}

		bool SerialFlowScheduleBase::open(const std::string &portname) {
			if(!is_open()) {
				if(!portname.empty()) {
					port->open(portname, *error_code);
				}
			}
			return is_open();
		}

		void SerialFlowScheduleBase::launch() {
			if(!port) {
				throw std::logic_error("Can not connect serial port");
			}
			port->get_io_service().post(
					[this]() {
						is_send_packet_empty() ? read() : write();
					}
			);
		}

		void SerialFlowScheduleBase::run() {
			port->get_io_service().run();
		}

		void SerialFlowScheduleBase::set_send_packet(const SinglePacket &send_packet) {
			std::lock_guard<std::mutex> lock(*mutex);
			send_packet_list.push_back(send_packet);
		}

		void SerialFlowScheduleBase::register_parse(ParseFunction functor) {
			parse_data_functor = functor;
		}

		void SerialFlowScheduleBase::set_timeout_ms(const unsigned int &timeout_ms) {
			this->timeout_ms = timeout_ms;
		}

		void SerialFlowScheduleBase::set_read_end_sleep_ms(const unsigned int &read_end_sleep_ms) {
			this->read_end_sleep_ms = std::chrono::milliseconds(read_end_sleep_ms);
		}

		void SerialFlowScheduleBase::set_write_end_sleep_ms(const unsigned int &write_end_sleep_ms) {
			this->write_end_sleep_ms = std::chrono::milliseconds(write_end_sleep_ms);
		}

		void SerialFlowScheduleBase::set_baudrate(const unsigned int &baudrate) {
			port->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
		}

		void SerialFlowScheduleBase::set_character_size(const unsigned int &size) {
			port->set_option(boost::asio::serial_port_base::character_size(size));
		}

		void SerialFlowScheduleBase::set_flow_control_none() {
			port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		}

		void SerialFlowScheduleBase::set_stop_bits_is_one() {
			port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		}

		void SerialFlowScheduleBase::set_parity_none() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		}

		void SerialFlowScheduleBase::set_parity_even() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
		}

		void SerialFlowScheduleBase::set_parity_odd() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
		}

		bool SerialFlowScheduleBase::is_send_packet_empty() {
			std::lock_guard<std::mutex> lock(*mutex);
			return send_packet_list.size() == 0;
		}

		void SerialFlowScheduleBase::read_timeout() {
			std::this_thread::yield();
			timer->expires_from_now(boost::posix_time::milliseconds(timeout_ms));
			timer->async_wait(std::bind(&SerialFlowScheduleBase::timeout_callback, this, std::placeholders::_1));
		}

		void SerialFlowScheduleBase::read() {
			read_timeout();
			port->async_read_some(
					boost::asio::buffer(read_buffer.data(), read_buffer.size()),
					std::bind(&SerialFlowScheduleBase::read_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
		}

		void SerialFlowScheduleBase::write() {
			if(is_send_packet_empty()) {
				throw std::runtime_error("Send packet is none");
			}
			boost::asio::async_write(
					*port,
					boost::asio::buffer(send_packet_list.front().data(), send_packet_list.front().size()),
					std::bind(&SerialFlowScheduleBase::write_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
		}

		void SerialFlowScheduleBase::initializer() {
			mutex = std::make_unique<std::mutex>();
			error_code = std::make_unique<boost::system::error_code>();
			timer = std::make_unique<boost::asio::deadline_timer>(port->get_io_service());
			read_end_sleep_ms = std::chrono::milliseconds(0);
			write_end_sleep_ms = std::chrono::milliseconds(0);
			clean_read_buffer();
			default_settings();
		}

		void SerialFlowScheduleBase::initializer(boost::asio::io_service &io_service) {
			port = std::make_unique<boost::asio::serial_port>(io_service);

			initializer();
		}

		void SerialFlowScheduleBase::initializer(boost::asio::serial_port &serial_port) {
			port = std::make_unique<boost::asio::serial_port>(std::move(serial_port));

			initializer();

			launch();
		}

		void SerialFlowScheduleBase::default_settings() {
			set_timeout_ms();
		}

		void SerialFlowScheduleBase::clean_read_buffer() {
			read_buffer.fill(Byte(0x00));
		}

		void SerialFlowScheduleBase::timeout_callback(const boost::system::error_code &error) {
			if(error) {
				error_code = std::make_unique<boost::system::error_code>(error);
			}
			else if(is_send_packet_empty()) {
				read_timeout();
			}
			else {
				port->cancel();
			}
		}

		void SerialFlowScheduleBase::read_callback(const boost::system::error_code &error, std::size_t bytes) {
			if(!error) {
				if(parse_data_functor) {
					parse_data_functor(read_buffer, bytes);
					clean_read_buffer();
				}
				read_end();
				if(is_send_packet_empty()) {
					read();
				}
				else {
					timer->cancel();
					write();
				}
			}
			else if(error == boost::asio::error::operation_aborted) {
				write();
			}
			else {
				error_code = std::make_unique<boost::system::error_code>(error);
				throw std::runtime_error(error.message());
			}
		}

		void SerialFlowScheduleBase::write_callback(const boost::system::error_code &error, std::size_t bytes) {
			if(!error) {
				if(bytes > 0) {
					send_packet_list.pop_front();
				}
				write_end();
				read();
			}
			else if(error == boost::asio::error::operation_aborted) {
				is_send_packet_empty() ? read() : write();
			}
			else {
				error_code = std::make_unique<boost::system::error_code>(error);
				throw std::runtime_error(error.message());
			}
		}

		void SerialFlowScheduleBase::read_end() {
			std::this_thread::sleep_for(read_end_sleep_ms);
		}
		
		void SerialFlowScheduleBase::write_end() {
			std::this_thread::sleep_for(write_end_sleep_ms);
		}
	}

}

