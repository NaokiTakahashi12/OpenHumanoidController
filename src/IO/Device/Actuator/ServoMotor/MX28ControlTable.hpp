
/**
  *
  * @file MX28ControlTable.hpp
  * @brief MX28 device control table
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include "../../../Communicator/Protocols/DynamixelVersion1.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				namespace MX28ControlTable {
					constexpr Communicator::Protocols::DynamixelVersion1::Byte
						model_number_low = 0x00, 				// eeprom start
						model_number_high = 0x01,
						version_of_firmware = 0x02,
						id = 0x03,
						baud_rate = 0x04,
						return_delay_time = 0x05,
						cw_angle_limit_low = 0x06,
						cw_angle_limit_high = 0x07,
						ccw_angle_limit_low = 0x08,
						ccw_angle_limit_high = 0x09,
						the_highest_limit_temperature = 0x0b,
						the_lowest_limit_voltage = 0x0c,
						the_highest_limit_voltage = 0x0d,
						max_torque_low = 0x0e,
						max_torque_high = 0x0f,
						status_return_level = 0x10,
						alarm_led = 0x11,
						alarm_shutdown = 0x12,
						multi_turn_offset_low = 0x14,
						multi_turn_offset_high = 0x15,
						resolution_divider = 0x16,				// eeprom end
						enable_torque = 0x18,					// ram start
						led = 0x19,
						derivative_gain = 0x1a,
						integral_gain = 0x1b,
						proportional_gain = 0x1c,
						goal_position_low = 0x1e,
						goal_position_high = 0x1f,
						moving_speed_low = 0x20,
						moving_speed_high = 0x21,
						torque_limit_low = 0x22,
						torque_limit_high = 0x23,
						present_position_low = 0x24,
						present_position_high = 0x25,
						present_speed_low = 0x26,
						present_speed_high = 0x27,
						present_load_low = 0x28,
						present_load_high = 0x29,
						present_voltage = 0x2a,
						present_temperature = 0x2b,
						registered = 0x2c,
						moving = 0x2e,
						lock = 0x2f,
						punch_low = 0x30,
						punch_high = 0x31,
						goal_acceleration = 0x49;				// ram end
				}
			}
		}
	}
}

