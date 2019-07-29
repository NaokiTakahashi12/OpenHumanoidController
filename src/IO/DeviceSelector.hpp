
#pragma once

#include <Tools/Log/Logger.hpp>

namespace IO {
	class DeviceSelector final {
		public :
			DeviceSelector();
			DeviceSelector(Tools::Log::LoggerPtr &);

		private :
			std::string config_filename;

			Tools::Log::LoggerPtr logger_ptr;
	};
}
