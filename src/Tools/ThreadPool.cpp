
#include "ThreadPool.hpp"

#include <exception>

namespace Tools {
	template <size_t N>
	ThreadPool<N>::ThreadPool() {
	}

	template <size_t N>
	void ThreadPool<N>::create_threads() {
		threads_ptr = std::make_unique<ThreadsType>();
	}

	template <size_t N>
	template <typename T>
	void ThreadPool<N>::regist(const std::function<T> &function_object) {
	}
}

