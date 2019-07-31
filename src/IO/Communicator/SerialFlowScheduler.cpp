
/**
  *
  * @file SerialFlowScheduler.cpp
  * @auther Naoki Takahashi
  *
  **/

#include "SerialFlowScheduler.hpp"

namespace IO {
	namespace Communicator {
		SerialFlowScheduler::SerialFlowScheduler() : SerialFlowScheduleBase() {
		}

		SerialFlowScheduler::~SerialFlowScheduler() {
			close();
		}
	}
}

