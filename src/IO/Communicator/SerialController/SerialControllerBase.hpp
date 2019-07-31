
/**
  *
  * @file SerialControllerBase.hpp
  * @brief Serial communication controller class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <RobotStatus/Information.hpp>

#include "../SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class SerialControllerBase {
				protected :
					using Thread = std::thread;
					using BaudRate = Communicator::SerialFlowScheduler::BaudRate;
					using ReadBuffer = Communicator::SerialFlowScheduler::ReadBuffer;
					using Length = Communicator::SerialFlowScheduler::Length;
					using SendPacket = Communicator::SerialFlowScheduler::SinglePacket;
					using ParseFunction = Communicator::SerialFlowScheduler::ParseFunction;

					std::unique_ptr<Thread> async_launch_thread;
					std::unique_ptr<SerialFlowScheduler> serial_flow_scheduler;
					RobotStatus::InformationPtr robo_info;

					virtual ParseFunction create_data_parser();

				public :
					SerialControllerBase();
					SerialControllerBase(RobotStatus::InformationPtr &);
					virtual ~SerialControllerBase();

					void port_name(const std::string &);
					std::string &port_name() const;

					void baud_rate(const BaudRate &);
					BaudRate &baud_rate() const;

					virtual void launch(),
								 async_launch();

					void register_parse(ParseFunction);

					void set_packet(const SendPacket &);

					void wait_for_send_packets() const;

				private :
					std::unique_ptr<BaudRate> baudrate;
					std::unique_ptr<std::string> device_port_name;
			};
		}
	}
}
