
/**
  *
  * @file MatrixOperation.hpp
  * @brief matlab like functions
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "Matrix.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Tools {
	namespace Math {
		template <typename T>
		T norm(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix);

		template <typename T>
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> diagonal(const Eigen::Matrix<T, Eigen::Dynamic, 1> &vector);
		template <typename T>
		MatrixX<T> diagonal(const VectorX<T> &vector);

		template <typename T>
		T dot(const Eigen::Matrix<T, Eigen::Dynamic, 1> &vector1, const Eigen::Matrix<T, Eigen::Dynamic, 1> &vector2);

		template <typename T>
		Eigen::Matrix<T, 3, 1> cross(const Eigen::Matrix<T, 3, 1> &vector1, const Eigen::Matrix<T, 3, 1> &vector2);
	}
}

