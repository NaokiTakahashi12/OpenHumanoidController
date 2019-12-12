
#pragma once

namespace Tools {
	namespace Math {
		namespace MathLiterals {
			float operator"" _\u03C0(unsigned long long count);
			float operator"" _deg(unsigned long long rad);
			float operator"" _rad(unsigned long long deg);
			float operator"" _k(unsigned long long value);
			float operator"" _M(unsigned long long value);
			float operator"" _\u03C0(long double count);
			float operator"" _deg(long double rad);
			float operator"" _rad(long double deg);
			float operator"" _k(long double value);
			float operator"" _M(long double value);
		}
	}
}

