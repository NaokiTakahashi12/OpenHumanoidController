
/**
  *
  * @file SerialFlowScheduler.hpp
  * @brief Serial communication management class
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include "SerialFlowScheduleBase.hpp"

#include <memory>

namespace IO {
	namespace Communicator {
		class SerialFlowScheduler final : public SerialFlowScheduleBase {
			public :
				SerialFlowScheduler();

				SerialFlowScheduler(const SerialFlowScheduler &) = delete;

				~SerialFlowScheduler();
		};
	}
}

