
/**
  *
  * @file ConvertStdVector.cpp
  *
  **/

#include <Eigen/Geometry>

#include "StdVector.hpp"

namespace Tools {
	namespace Converter {
		namespace StdVector {
			template <>
			Eigen::Matrix<int, Eigen::Dynamic, 1> to_vector(std::vector<int> &vector_stl) {
				return Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, 1>, Eigen::Unaligned>(vector_stl.data(), vector_stl.size()).matrix();
			}

			template <>
			Eigen::Matrix<float, Eigen::Dynamic, 1> to_vector(std::vector<float> &vector_stl) {
				return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>, Eigen::Unaligned>(vector_stl.data(), vector_stl.size()).matrix();
			}

			template <>
			Eigen::Matrix<double, Eigen::Dynamic, 1> to_vector(std::vector<double> &vector_stl) {
				return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>, Eigen::Unaligned>(vector_stl.data(), vector_stl.size()).matrix();
			}
		}
	}
}

