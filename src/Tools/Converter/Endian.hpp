
#pragma once

namespace Tools {
	namespace Converter {
		namespace Endian {
			template <typename T>
			T swap(const T &);

			bool is_big_endian();
			bool is_little_endian();
		}
	}
}

