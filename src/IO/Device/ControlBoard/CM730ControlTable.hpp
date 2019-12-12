
/**
  *
  * @file CM730ControlTable.hpp
  * @brief CM730 control table
  * @auther Naoki Takahashi
  *
  **/

#pragma once

#include "../../Communicator/Protocols/DynamixelVersion1.hpp"

namespace IO {
	namespace Device {
		namespace ControlBoard {
			namespace CM730ControlTable {
				constexpr Communicator::Protocols::DynamixelVersion1::Byte
					model_number_low = 0x00, 				// eeprom start
					model_number_high = 0x01,
					version_of_firmware = 0x02,
					id = 0x03,
					baud_rate = 0x04,
					return_delay_time = 0x05,
					status_return_level = 0x10,				// eeprom end
					dynamixel_power = 0x18,					// ram start
					enable_led_pannel = 0x19,
					led_5_low = 0x1a,
					led_5_high = 0x1b,
					led_6_low = 0x1c,
					led_6_high = 0x1d,
					button = 0x1e,
					gyro_z_low = 0x26,
					gyro_z_high = 0x27,
					gyro_y_low = 0x28,
					gyro_y_high = 0x29,
					gyro_x_low = 0x2a,
					gyro_x_high = 0x2b,
					acc_x_low = 0x2c,
					acc_x_high = 0x2d,
					acc_y_low = 0x2e,
					acc_y_high = 0x2f,
					acc_z_low = 0x30,
					acc_z_high = 0x31,
					present_voltage = 0x32,
					mic_low = 0x33,
					mic_high = 0x34,
					adc_2_low = 0x35,
					adc_2_high = 0x36,
					adc_3_low = 0x37,
					adc_3_high = 0x38,
					adc_4_low = 0x39,
					adc_4_high = 0x3a,
					adc_5_low = 0x3b,
					adc_5_high = 0x3c,
					adc_6_low = 0x3d,
					adc_6_high = 0x3e,
					adc_7_low = 0x3f,
					adc_7_high = 0x40,
					adc_8_low = 0x41,
					adc_8_high = 0x41,
					adc_9_low = 0x42,
					adc_9_high = 0x43,
					adc_10_low = 0x44,
					adc_10_high = 0x45,
					adc_11_low = 0x46,
					adc_11_high = 0x47,
					adc_12_low = 0x48,
					adc_12_high = 0x49,
					adc_13_low = 0x4a,
					adc_13_high = 0x4b,
					adc_14_low = 0x4c,
					adc_14_high = 0x4d,
					adc_15_low = 0x4e,
					adc_15_high = 0x4f;						// ram end
			}
		}
	}
}

