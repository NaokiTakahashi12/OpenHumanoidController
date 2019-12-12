
/**
  *
  * @file Byte.cpp
  * @Naoki Takahashi
  *
  **/

#include "Byte.hpp"

#include <cstdint>
#include <cassert>
#include <cstddef>

namespace Tools {
	namespace Byte {
		constexpr size_t maximum_support_byte_lenght = 2;

		template <typename T>
		T low_byte(const T &x) {
			static_assert(sizeof(x) == maximum_support_byte_lenght, "Size over from Tools::Byte");
			return static_cast<T>(x & 0x00ff);
		}

		template <typename T>
		T high_byte(const T &x) {
			static_assert(sizeof(x) == maximum_support_byte_lenght, "Size over from Tools::Byte");
			return static_cast<T>((x & 0xff00) >> 8);
		}

		template <typename T>
		T union_byte(const T &high, const T &low) {
			static_assert(sizeof(high) == maximum_support_byte_lenght || sizeof(low) == maximum_support_byte_lenght, "Size over from Tools::Byte");
			return (high << 8) + low;
		}

		template short low_byte<short>(const short &);
		template unsigned short low_byte<unsigned short>(const unsigned short &);

		template short high_byte<short>(const short &);
		template unsigned short high_byte<unsigned short>(const unsigned short &);

		template short union_byte<short>(const short &, const short &);
		template unsigned short union_byte<unsigned short>(const unsigned short &, const unsigned short &);
	}
}

