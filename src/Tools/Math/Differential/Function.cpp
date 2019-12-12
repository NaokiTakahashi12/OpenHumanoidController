
#include "Function.hpp"

namespace Tools {
	namespace Math {
		namespace Differential {
			template <typename T>
			Function<T>::Function() {
			}

			template <typename T>
			Function<T>::~Function() {
			}

			template <typename T>
			T Function<T>::get_value() const {
				return 0x11;
			}

			template class Function<float>;
			template class Function<double>;
		}
	}
}

