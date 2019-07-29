
/**
  *
  * @file SerialFlowScheduler.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		SerialFlowScheduler::SerialFlowScheduler(boost::asio::io_service &io_service) : SerialFlowScheduleBase(io_service) {
		}

		SerialFlowScheduler::SerialFlowScheduler(boost::asio::serial_port &serial_port) : SerialFlowScheduleBase(serial_port) {
		}

		SerialFlowScheduler::~SerialFlowScheduler() {
			close();
		}
	}
}

