
/**
  *
  * @file SerialControlBoard.hpp
  * @brief Control board utile class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../SerialDeviceBase.hpp"

#include <RobotStatus/Information.hpp>

#include "../../Communicator/SerialReturnPacket.hpp"
#include "../../Communicator/SerialController/SerialControllerBase.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			class SerialControlBoard : public SerialDeviceBase<Communicator::SerialController::SerialControllerBase> {
				protected :
					using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
					using WriteValue = Communicator::SerialFlowScheduler::Byte;

				public :
					using ID = Communicator::SerialReturnPacket::PacketID;

					SerialControlBoard(RobotStatus::InformationPtr &);
					SerialControlBoard(const ID &, RobotStatus::InformationPtr &);
					virtual ~SerialControlBoard();

					virtual void ping(),
								 enable_power(const bool &flag);

					ID &id() const;
					void id(const ID &);

				protected :
					RobotStatus::InformationPtr robo_info;

				private :
					std::unique_ptr<ID> device_id;
			};
		}
	}
}

