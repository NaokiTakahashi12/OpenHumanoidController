
#include <sstream>

#include "MatrixToString.hpp"

namespace Tools {
	namespace Converter {
		template <typename T>
		std::string matrix_to_string(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix) {
			std::stringstream ss;
			ss << matrix;

			return ss.str();
		}

		template <typename T>
		std::string matrix_to_string(T &vector) {
			std::stringstream ss;
			ss << vector;

			return ss.str();
		}

		template std::string matrix_to_string<int>(Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> &matrix);
		template std::string matrix_to_string<float>(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &matrix);
		template std::string matrix_to_string<double>(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &matrix);

		template std::string matrix_to_string<Eigen::VectorXi>(Eigen::VectorXi &);
		template std::string matrix_to_string<Eigen::VectorXf>(Eigen::VectorXf &);
		template std::string matrix_to_string<Eigen::VectorXd>(Eigen::VectorXd &);
	}
}
