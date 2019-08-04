
/**
  *
  * @file VMU931.hpp
  * @brief VMU931 (IMU) serial communicator class
  * @author Naoki Takahashi
  *
  **/

#pragma once

#include "../../../Communicator/SerialController/Simple.hpp"
#include "InertialMeasurementUnit.hpp"

namespace IO {
	namespace Device {
		namespace Sensor {
			namespace IMU {
				//! @todo Catch string message
				class VMU931 final : public InertialMeasurementUnit {
					public :
						VMU931(RobotStatus::InformationPtr &);

						void enable(const Streams &) override final;
						void enable_all() override final;

						void launch() override final;
						void async_launch() override final;

					private :
						using ReadBuffer = Communicator::SerialFlowScheduler::ReadBuffer;
						using Length = Communicator::SerialFlowScheduler::Length;

						static constexpr uint8_t data_start_bit = 0x01,
								  		  		 data_end_bit = 0x04;

						static constexpr auto streams_prefix = "var";

						RobotStatus::InformationPtr robo_info;

						std::unique_ptr<Communicator::SerialController::Simple> simple_serial_controller;

						Communicator::SerialFlowScheduler::SinglePacket streaming_list;

						void create_data_space(const Streams &);

						template <typename T>
						T &single_value_cast(const ReadBuffer &read_buffer, const uint8_t &position);

						template <typename T>
						T &timestamp_parser(const ReadBuffer &read_buffer, const uint8_t &head_position);

						float &heading_parser(const ReadBuffer &read_buffer, const uint8_t &head_position);
						Tools::Math::Vector3<float> &vector3_parser(const ReadBuffer &read_buffer, const uint8_t &head_position);
						Tools::Math::Vector4<float> &vector4_parser(const ReadBuffer &read_buffer, const uint8_t &head_position);

					 	void packet_parse(const ReadBuffer &read_buffer, const uint8_t &head_position);

						bool packet_splitter(const ReadBuffer &, const Length &);

						bool broken_packet_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position),
							 constant_bit_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position),
							 end_packet_checker(const ReadBuffer &read_buffer, const Length &bytes, const uint8_t &head_position);
				};
			}
		}
	}
}

