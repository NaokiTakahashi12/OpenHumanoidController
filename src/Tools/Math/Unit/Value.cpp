
#include "Value.hpp"

#include <stdexcept>

namespace Tools {
	namespace Math {
		namespace Unit {
			template <typename T, SI UNIT>
			Value<T, UNIT>::Value(const T &x) {
				scholar = std::make_unique<T>(x);
			}

			template <typename T, SI UNIT>
			Value<T, UNIT>::Value(const VectorX<T> &x) {
				vector = std::make_unique<VectorX<T>>(x);
			}

			template <typename T, SI UNIT>
			Value<T, UNIT>::Value(const MatrixX<T> &x) {
				matrix = std::make_unique<MatrixX<T>>(x);
			}

			template <typename T, SI UNIT>
			T Value<T, UNIT>::get() {
				if(scholar) {
					return *scholar;
				}
				throw std::runtime_error("Can not access scholar type");
			}

			template class Value<int, SI::metre>;
			template class Value<float, SI::metre>;
			template class Value<double, SI::metre>;
		}
	}
}

