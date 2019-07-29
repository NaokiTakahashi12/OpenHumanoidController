
/**
  *
  * @file Byte.hpp
  * @Naoki Takahashi
  *
  **/

#pragma once

namespace Tools {
	namespace Byte {
		template <typename T>
		T low_byte(const T &);

		template <typename T>
		T high_byte(const T &);

		template <typename T>
		T union_byte(const T &high, const T &low);
	}
}

