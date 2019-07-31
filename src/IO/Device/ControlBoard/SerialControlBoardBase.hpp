
/**
  *
  * @file SerialControlBoardBase.hpp
  * @brief Serial communicatoin basic class for control board device
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "../../Communicator/SerialReturnPacket.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			class SerialControlBoardBase {
				public :
					using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
					using WriteValue = Communicator::SerialFlowScheduler::Byte;
					using ID = Communicator::SerialReturnPacket::PacketID;

					SerialControlBoardBase(RobotStatus::InformationPtr &);
					SerialControlBoardBase(const ID &, RobotStatus::InformationPtr &);
					virtual ~SerialControlBoardBase();

					ID &id() const;
					void id(const ID &) const;

				protected :
					RobotStatus::InformationPtr robo_info;

				private :
					std::unique_ptr<ID> device_id;
			};
		}
	}
}

