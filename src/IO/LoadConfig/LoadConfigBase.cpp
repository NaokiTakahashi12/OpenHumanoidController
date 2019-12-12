
/**
  *
  * @file LoadConfigBase.cpp
  * @author Naoki Takahashi
  *
  **/

#include "LoadConfigBase.hpp"

#include <stdexcept>

namespace IO {
	namespace LoadConfig {
		LoadConfigBase::~LoadConfigBase() {
		}

		void LoadConfigBase::update() {
			throw std::runtime_error("Unoverride");
		}

		void LoadConfigBase::force_update() {
			throw std::runtime_error("Unoverride");
		}
	}
}

