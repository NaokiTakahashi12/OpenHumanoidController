
#pragma once

namespace Tools {
	class NonCopyable {
		protected :
			NonCopyable();
			~NonCopyable();

		private :
			NonCopyable(const NonCopyable &);
			NonCopyable &operator = (const NonCopyable &);
	};
}
