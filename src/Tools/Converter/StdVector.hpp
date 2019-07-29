
/**
  *
  * @file ConvertStdVector.hpp
  *
  **/

#pragma once

#include <vector>

#include <Eigen/Dense>

namespace Tools {
	namespace Converter {
			namespace StdVector {
			template <typename T>
			Eigen::Matrix<T, Eigen::Dynamic, 1> to_vector(std::vector<T> &);
		}
	}
}

