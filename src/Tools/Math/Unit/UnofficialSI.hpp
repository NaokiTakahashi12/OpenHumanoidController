
#pragma once

#include "SI.hpp"

namespace Tools {
	namespace Math {
		namespace Unit {
			enum class UnofficialSI : char {
				minute = static_cast<char>(SI::candela) + 1,
				hour,
				day,
				radian,
				degree,
			};
		}
	}
}

