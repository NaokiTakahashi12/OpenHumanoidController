
/**
  *
  * @file SerialReturnPacket
  * @brief Serial communication return packet structure
  * @author NaokiTakahashi
  *
  **/

#pragma once

#include <deque>

#include "SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		struct SerialReturnPacket {
			using PacketID = SerialFlowScheduler::Byte;
			using ContentsType = SerialFlowScheduler::Byte;
			using PacketContents = std::deque<ContentsType>;

			PacketID id;
			PacketContents contents;
		};
	}
}

