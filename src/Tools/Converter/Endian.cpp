
#include "Endian.hpp"

#include <cstdint>

namespace Tools {
	namespace Converter {
		namespace Endian {
			template <>
			uint32_t swap<uint32_t>(const uint32_t &value) {
				static uint32_t ret;

				ret = value << 24;
				ret |= (value & 0x0000ff00) << 8;
				ret |= (value & 0x00ff0000) >> 8;
				ret |= value >> 24;

				return ret;
			}

			template <>
			uint16_t swap<uint16_t>(const uint16_t &value) {
				static uint16_t ret;

				ret = value << 8;
				ret |= value >> 8;

				return ret;
			}

			bool is_big_endian() {
				constexpr int x = 1;
				return *(char *)&x ? false : true;
			}

			bool is_little_endian() {
				constexpr int x = 1;
				return *(char *)&x ? true : false;
			}
		}
	}
}

