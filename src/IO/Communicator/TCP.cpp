
/**
  *
  * @file TCP.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "TCP.hpp"

namespace IO {
	namespace Communicator {
		TCP::TCP(const short &port) {
			io_service = std::make_unique<boost::asio::io_service>();
			this->port = std::make_unique<short>(port);

			socket = std::make_unique<boost::asio::ip::tcp::socket>(*io_service);
			error_code = std::make_unique<boost::system::error_code>();
		}

		TCP::~TCP() {
			if(acceptor != nullptr) {
				acceptor->close();
			}

			socket->close();
		}

		boost::system::error_code &TCP::server_v4() {
			endpoint = std::make_unique<boost::asio::ip::tcp::endpoint>(boost::asio::ip::tcp::v4(), *port);

			return accept();
		}

		boost::system::error_code &TCP::server_v6() {
			endpoint = std::make_unique<boost::asio::ip::tcp::endpoint>(boost::asio::ip::tcp::v6(), *port);

			return accept();
		}

		std::string TCP::read() {
			boost::asio::streambuf buffer;

			boost::asio::read(*socket, buffer, boost::asio::transfer_at_least(true), *error_code);

			return boost::asio::buffer_cast<const char*>(buffer.data());
		}

		boost::system::error_code &TCP::read(std::string &read_string) {
			boost::asio::streambuf buffer;

			boost::asio::read(*socket, buffer, boost::asio::transfer_at_least(true), *error_code);
			read_string = boost::asio::buffer_cast<const char*>(buffer.data());

			return *error_code;
		}

		boost::system::error_code &TCP::client(const std::string &address) {
			endpoint = std::make_unique<boost::asio::ip::tcp::endpoint>(boost::asio::ip::address::from_string(address), *port);

			return connect();
		}

		boost::system::error_code &TCP::write(const std::string &buffer) {
			boost::asio::write(*socket, boost::asio::buffer(buffer));

			return *error_code;
		}

		boost::system::error_code &TCP::accept() {
			acceptor = std::make_unique<boost::asio::ip::tcp::acceptor>(*io_service, *endpoint);
			acceptor->accept(*socket);

			return *error_code;
		}

		boost::system::error_code &TCP::connect() {
			socket->connect(*endpoint, *error_code);

			return *error_code;
		}
	}
}

