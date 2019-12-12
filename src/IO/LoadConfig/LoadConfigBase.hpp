
/**
  *
  * @file LoadConfigBase.hpp
  * @brief load config base class
  * @author Naoki Takahashi
  *
  **/

#pragma once

namespace IO {
	namespace LoadConfig {
		class LoadConfigBase {
			public :
				virtual ~LoadConfigBase();
				virtual void update();
				virtual void force_update();
		};
	}
}

