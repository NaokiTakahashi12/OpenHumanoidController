
#include "MathLiterals.hpp"
#include "../Constant.hpp"

namespace Tools {
	namespace Math {
		namespace MathLiterals {
			float operator"" _\u03C0(unsigned long long count) {
				return 3.141592 * count;
			}

			float operator"" _deg(unsigned long long rad) {
				return Constant::radian_to_degree * rad;
			}

			float operator"" _rad(unsigned long long deg) {
				return Constant::degree_to_radian * deg;
			}

			float operator"" _k(unsigned long long value) {
				return 1000 * value;
			}

			float operator"" _M(unsigned long long value) {
				return 1000000 * value;
			}
			float operator"" _\u03C0(long double count) {
				return 3.141592 * count;
			}

			float operator"" _deg(long double rad) {
				return Constant::radian_to_degree * rad;
			}

			float operator"" _rad(long double deg) {
				return Constant::degree_to_radian * deg;
			}

			float operator"" _k(long double value) {
				return 1000 * value;
			}

			float operator"" _M(long double value) {
				return 1000000 * value;
			}
		}
	}
}

