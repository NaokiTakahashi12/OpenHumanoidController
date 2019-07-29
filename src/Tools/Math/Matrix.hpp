
#pragma once

#include <Eigen/Dense>

namespace Tools {
	namespace Math {
		template <typename T, int R, int C>
		using Matrix = Eigen::Matrix<T, R, C>;

		template <typename T>
		using Matrix2 = Matrix<T, 2, 2>;
		template <typename T>
		using Matrix3 = Matrix<T, 3, 3>;
		template <typename T>
		using Matrix4 = Matrix<T, 4, 4>;

		template <typename T>
		using MatrixX = Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

		template <typename T, int R>
		using Vector = Matrix<T, R, 1>;

		template <typename T>
		using Vector2 = Vector<T, 2>;
		template <typename T>
		using Vector3 = Vector<T, 3>;
		template <typename T>
		using Vector4 = Vector<T, 4>;

		template <typename T>
		using VectorX = Vector<T, Eigen::Dynamic>;
	}
}

