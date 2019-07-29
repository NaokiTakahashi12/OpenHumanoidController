
#pragma once

#include <memory>

#include "../Matrix.hpp"

#include "SI.hpp"
#include "UnofficialSI.hpp"

namespace Tools {
	namespace Math {
		namespace Unit {
			template <typename T, SI UNIT>
			class Value {
				public :
					Value(const T &);
					Value(const VectorX<T> &);
					Value(const MatrixX<T> &);

					T get();

				private :
					std::unique_ptr<T> scholar;
					std::unique_ptr<VectorX<T>> vector;
					std::unique_ptr<MatrixX<T>> matrix;
			};
		}
	}
}

