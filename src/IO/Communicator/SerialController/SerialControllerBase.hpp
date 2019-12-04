
/**
  *
  * @file SerialControllerBase.hpp
  * @brief Serial communication controller class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include <mutex>
#include <unordered_map>
#include <thread>
#include <memory>

#include "../SerialReturnPacket.hpp"
#include "../SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			class SerialControllerBase {
				protected :
					using ReadBuffer = Communicator::SerialFlowScheduler::ReadBuffer;
					using Length = Communicator::SerialFlowScheduler::Length;
					using Thread = std::thread;

				public :
					using BaudRate = SerialFlowScheduler::BaudRate;
					using TimeoutMs = unsigned int;
					using SendPacket = SerialFlowScheduler::SinglePacket;
					using ParseFunction = SerialFlowScheduler::ParseFunction;
					using ReturnPacketMap = std::unordered_map<SerialReturnPacket::PacketID, SerialReturnPacket>;

					SerialControllerBase();
					virtual ~SerialControllerBase();

					void port_name(const std::string &);
					std::string &port_name() const;

					void baud_rate(const BaudRate &);
					BaudRate &baud_rate() const;

					void timeout_ms(const TimeoutMs &);
					TimeoutMs &timeout_ms() const;

					virtual void launch(),
								 async_launch();

					void register_parse(ParseFunction);

					void set_packet(const SendPacket &);

					void wait_for_send_packets() const;

					SerialReturnPacket &return_packet(const SerialReturnPacket::PacketID &) const;
					bool is_exist_return_packet(const SerialReturnPacket::PacketID &) const;

					ReturnPacketMap &access_return_packet_map();

				protected :
					std::unique_ptr<Thread> async_launch_thread;
					std::unique_ptr<SerialFlowScheduler> serial_flow_scheduler;
					std::unique_ptr<ReturnPacketMap> return_packet_map;

					virtual ParseFunction create_data_parser();

				private :
					std::unique_ptr<std::mutex> data_access_mutex;
					std::unique_ptr<BaudRate> baudrate;
					std::unique_ptr<TimeoutMs> timeoutms;
					std::unique_ptr<std::string> device_port_name;
			};
		}
	}
}
