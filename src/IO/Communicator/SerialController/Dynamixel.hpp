
/**
  *
  * @file Dynamixel.hpp
  * @beief Dynamixel communication control class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "SerialControllerBase.hpp"

#include <thread>
#include <memory>
#include <unordered_map>
#include <vector>
#include <deque>

#include <Tools/Log/Logger.hpp>
#include <Tools/NonCopyable.hpp>

#include "../SerialReturnPacket.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class Dynamixel final : public SerialControllerBase, Tools::NonCopyable {
				private :
					struct DynamixelData : SerialReturnPacket {
						using ErrorCode = SerialFlowScheduler::Byte;
						ErrorCode error_code;
					};

					using ID = SerialFlowScheduler::Byte;
					using IDList = std::vector<ID>;
					using DynamixelDataMap = std::unordered_map<ID, DynamixelData>;

					DynamixelDataMap data_map;

					ParseFunction create_data_parser() override;
					
					bool packet_splitter(const ReadBuffer &, const Length &);
					void data_parser(const ReadBuffer &, const unsigned int &);

				public :
					Dynamixel();
					~Dynamixel();

					void launch() override;
					void async_launch() override;

					DynamixelData catch_packet(const ID &);
					IDList catch_packet_id();
					bool is_exist(const ID &);
			};
			using DynamixelPtr = std::shared_ptr<Dynamixel>;
		}
	}
}

