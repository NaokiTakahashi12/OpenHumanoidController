
#pragma once

#include <string>

#include <Eigen/Dense>

namespace Tools {
	namespace Converter {
		template <typename T>
		std::string matrix_to_string(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix);

		template <typename T>
		std::string matrix_to_string(T &vector);
	}
}

