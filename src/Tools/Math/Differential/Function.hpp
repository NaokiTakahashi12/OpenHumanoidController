
#pragma once

#include "../Matrix.hpp"

namespace Tools {
	namespace Math {
		namespace Differential {
			template <typename T>
			class Function {
				public :
					Function();
					virtual ~Function();

					T get_value() const;

				private :

			};
		}
	}
}

