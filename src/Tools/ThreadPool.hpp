
#pragma once

#include <array>
#include <memory>
#include <functional>
#include <thread>

namespace Tools {
	template <size_t N>
	class ThreadPool {
		protected :
			using ThreadsType = std::array<std::thread, N>;
			using ThreadsPtrType = std::unique_ptr<ThreadsType>;

		public :
			ThreadPool();

			template <typename T>
			void regist(const std::function<T> &);

		private :
			ThreadsPtrType threads_ptr;

			void create_threads();
	};
}

