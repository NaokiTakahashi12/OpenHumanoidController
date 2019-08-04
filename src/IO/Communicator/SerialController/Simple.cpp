
 /**
   *
   * @file Simple.cpp
   * @author Naoki Takahashi
   *
   **/

#include "Simple.hpp"

namespace IO {
	namespace Communicator {
		namespace SerialController {
			Simple::~Simple() {
				if(serial_flow_scheduler) {
					serial_flow_scheduler->close();
				}
				if(async_launch_thread) {
					async_launch_thread->join();
				}
			}

			void Simple::launch() { 
				if(!serial_flow_scheduler) {
					throw std::runtime_error("Not exist SerialFlowScheduler from IO::Communicator::SerialController::Simple");
				}
				serial_flow_scheduler->open(port_name());
				serial_flow_scheduler->set_baudrate(baud_rate());
				serial_flow_scheduler->launch();
			}

			void Simple::async_launch() {
				if(async_launch_thread) {
					throw std::runtime_error("Already regist thread from IO::Communicator::SerialController::Simple");
				}
				async_launch_thread = std::make_unique<Thread>(&Simple::launch, this);
			}
		}
	}
}

