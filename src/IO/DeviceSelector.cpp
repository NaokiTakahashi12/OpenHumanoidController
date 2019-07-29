
#include "DeviceSelector.hpp"

namespace IO {
	DeviceSelector::DeviceSelector() {
		if(!logger_ptr) {
			logger_ptr = std::make_unique<Tools::Log::Logger>();
		}
	}

	DeviceSelector::DeviceSelector(Tools::Log::LoggerPtr &logger) {
		logger_ptr = logger;
		DeviceSelector();
	}
}

