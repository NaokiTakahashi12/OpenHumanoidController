
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
		SerialFlowScheduleBase::SerialFlowScheduleBase() {
			packet_mutex = std::make_unique<std::mutex>();
			io_service = std::make_unique<boost::asio::io_service>();
			port = std::make_unique<boost::asio::serial_port>(*io_service);
			error_code = std::make_unique<boost::system::error_code>();
			timer = std::make_unique<boost::asio::deadline_timer>(*io_service);
			read_end_sleep_us = std::chrono::microseconds(0);
			write_end_sleep_us = std::chrono::microseconds(0);
			clean_read_buffer();
			default_settings();
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

		bool SerialFlowScheduleBase::is_open() const {
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
			io_service->post(
					[this]() {
						is_send_packet_empty() ? read() : write();
					}
			);
			io_service->run();
		}

		void SerialFlowScheduleBase::set_send_packet(const SinglePacket &send_packet) {
			auto lock = std::lock_guard<std::mutex>(*packet_mutex);
			send_packet_list.push_back(send_packet);
		}

		void SerialFlowScheduleBase::register_parse(ParseFunction functor) {
			parse_data_functor = functor;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_timeout_us(const unsigned int &timeout_us) {
			this->timeout_us = timeout_us;
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_read_end_sleep_us(const unsigned int &read_end_sleep_us) {
			this->read_end_sleep_us = std::chrono::microseconds(read_end_sleep_us);
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_write_end_sleep_us(const unsigned int &write_end_sleep_us) {
			this->write_end_sleep_us = std::chrono::microseconds(write_end_sleep_us);
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_baudrate(const BaudRate &baudrate) {
			port->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_character_size(const unsigned int &size) {
			port->set_option(boost::asio::serial_port_base::character_size(size));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_flow_control_none() {
			port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_stop_bits_is_one() {
			port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_parity_none() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_parity_even() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
			return *this;
		}

		SerialFlowScheduleBase &SerialFlowScheduleBase::set_parity_odd() {
			port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
			return *this;
		}

		bool SerialFlowScheduleBase::is_send_packet_empty() const {
			auto lock = std::lock_guard<std::mutex>(*packet_mutex);
			return send_packet_list.size() == 0;
		}

		void SerialFlowScheduleBase::read_timeout() {
			std::this_thread::yield();
			timer->expires_from_now(boost::posix_time::microseconds(timeout_us));
			timer->async_wait(std::bind(&SerialFlowScheduleBase::timeout_callback, this, std::placeholders::_1));
		}

		void SerialFlowScheduleBase::read() {
			read_timeout();
			port->async_read_some(
					boost::asio::buffer(read_buffer.data(), read_buffer.size()),
					std::bind(&SerialFlowScheduleBase::read_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
#if 0
			printf("READ: ");
			for(int i = 0; i < read_buffer.data()[0];i ++) {
				printf("%02X ", read_buffer.data()[i] & 0xFF);
			}
			printf("\r\n");
#endif
		}

		void SerialFlowScheduleBase::write() {
			if(is_send_packet_empty()) {
				throw std::runtime_error("Send packet is none from IO::Communicator::SerialFlowScheduleBase");
			}
			boost::asio::async_write(
					*port,
					boost::asio::buffer(send_packet_list.front().data(), send_packet_list.front().size()),
					std::bind(&SerialFlowScheduleBase::write_callback, this, std::placeholders::_1, std::placeholders::_2)
			);
#if 0
			printf("SEND: ");
			for(int i = 0; i < send_packet_list.front().size();i ++) {
				printf("%02X ", send_packet_list.front().data()[i] & 0xFF);
			}
			printf("\r\n");
#endif
		}

		void SerialFlowScheduleBase::default_settings() {
			set_timeout_us();
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

		void SerialFlowScheduleBase::read_callback(const boost::system::error_code &error, Length bytes) {
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

		void SerialFlowScheduleBase::write_callback(const boost::system::error_code &error, Length bytes) {
			if(!error) {
				if(bytes > 0) {
					auto lock = std::lock_guard<std::mutex>(*packet_mutex);
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
			std::this_thread::sleep_for(read_end_sleep_us);
		}
		
		void SerialFlowScheduleBase::write_end() {
			std::this_thread::sleep_for(write_end_sleep_us);
		}
	}

}

