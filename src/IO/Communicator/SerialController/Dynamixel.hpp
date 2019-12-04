
/**
  *
  * @file Dynamixel.hpp
  * @beief Dynamixel communication control class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SerialControllerBase.hpp"

#include <memory>

#include <Tools/Log/Logger.hpp>

#include "../SerialReturnPacket.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class Dynamixel final : public SerialControllerBase {
				private :
					ParseFunction create_data_parser() override;
					
					bool packet_checker(const ReadBuffer &, const Length &length, const Length &head_position);
					bool packet_splitter(const ReadBuffer &, const Length &);
					void data_parser(const ReadBuffer &, const unsigned int &);

				public :
					Dynamixel();
					~Dynamixel();

					static std::string get_key();

					void launch() override;
					void async_launch() override;

			};
			using DynamixelPtr = std::shared_ptr<Dynamixel>;
		}
	}
}

