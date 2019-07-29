
/**
  *
  * @file MessageLevels.hpp
  * @brief Message levels
  * @author Naoki Takahashi
  *
  **/

#pragma once

namespace Tools {
	namespace Log {
		enum class MessageLevels : uint8_t {
			trace = 1,
			debug,
			info,
			warning,
			error,
			fatal
		};
	}
}

