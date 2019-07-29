
/**
  *
  * @file MatrixOperation.cpp
  * @author Naoki Takahashi
  *
  **/

#include "MatrixOperation.hpp"

namespace Tools {
	namespace Math {
		template <>
		int norm(const Eigen::MatrixXi &matrix) {
			return matrix.lpNorm<Eigen::Infinity>();
		}

		template <>
		float norm(const Eigen::MatrixXf &matrix) {
			return matrix.lpNorm<Eigen::Infinity>();
		}

		template <>
		double norm(const Eigen::MatrixXd &matrix) {
			return matrix.lpNorm<Eigen::Infinity>();
		}

		template <>
		Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> diagonal(const Eigen::VectorXi &vector) {
			return vector.asDiagonal();
		}

		template <>
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> diagonal(const Eigen::VectorXf &vector) {
			return vector.asDiagonal();
		}

		template <>
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> diagonal(const Eigen::VectorXd &vector) {
			return vector.asDiagonal();
		}

		template <>
		int dot(const Eigen::VectorXi &vector1, const Eigen::VectorXi &vector2) {
			return vector1.dot(vector2);
		}

		template <>
		float dot(const Eigen::VectorXf &vector1, const Eigen::VectorXf &vector2) {
			return vector1.dot(vector2);
		}

		template <>
		double dot(const Eigen::VectorXd &vector1, const Eigen::VectorXd &vector2) {
			return vector1.dot(vector2);
		}

		template <>
		Eigen::Matrix<int, 3, 1> cross(const Eigen::Matrix<int, 3, 1> &vector1, const Eigen::Matrix<int, 3, 1> &vector2) {
			return vector1.cross(vector2);
		}

		template <>
		Eigen::Matrix<float, 3, 1> cross(const Eigen::Matrix<float, 3, 1> &vector1, const Eigen::Matrix<float, 3, 1> &vector2) {
			return vector1.cross(vector2);
		}

		template <>
		Eigen::Matrix<double, 3, 1> cross(const Eigen::Matrix<double, 3, 1> &vector1, const Eigen::Matrix<double, 3, 1> &vector2) {
			return vector1.cross(vector2);
		}
	}
}

