
/**
  *
  * @file Kondo.hpp
  * @beief Kondo communication control class
  * @author Yasuo Hayashibara
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

#include "../SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class Kondo final : public SerialControllerBase {
				private :
					struct KondoData {
						SerialFlowScheduler::Byte error_code;
						std::vector<SerialFlowScheduler::Byte> parameters;
					};
					using ID = SerialFlowScheduler::Byte;
					using IDList = std::vector<ID>;
					using KondoDataMap = std::unordered_map<ID, KondoData>;

					KondoDataMap data_map;

					ParseFunction create_data_parser() override;
					
					bool data_parser(const ReadBuffer &, const Length &);

				public :
					Kondo();
					Kondo(RobotStatus::InformationPtr &);
					~Kondo();

					void launch() override;
					void async_launch() override;

					void packet(const SendPacket &);

					KondoData catch_packet(const ID &);
					IDList catch_packet_id();
					bool is_exist(const ID &);
			};
			using KondoPtr = std::shared_ptr<Kondo>;
		}
	}
}

