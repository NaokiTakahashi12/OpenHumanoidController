
/**
  *
  * @file TCP.hpp
  * @brief TCP communication class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include <memory>
#include <string>

#include <boost/asio.hpp>

namespace IO {
	namespace Communicator {
		class TCP {
			public :
				TCP(boost::asio::io_service &, const short &port);
				TCP(const TCP &) = delete;
				~TCP();

				boost::system::error_code &server_v4(),
										  &server_v6();

				std::string read();
				boost::system::error_code &read(std::string &read_string);

				boost::system::error_code &client(const std::string &address);

				boost::system::error_code &write(const std::string &buffer);

			private :
				std::unique_ptr<short> port;

				std::unique_ptr<boost::system::error_code> error_code;
				
				std::unique_ptr<boost::asio::ip::tcp::socket> socket;
				std::unique_ptr<boost::asio::ip::tcp::endpoint> endpoint;
				std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor;

				boost::system::error_code &accept(),
										  &connect();
		};
	}
}

