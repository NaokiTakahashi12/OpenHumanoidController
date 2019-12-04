
/**
  *
  * @file KondoB3MControlTable.hpp
  * @brief Kondo B3M device control table
  * @auther Yasuo Hayashibara
  *
  **/

#pragma once

#include "../../../Communicator/Protocols/KondoB3M.hpp"

namespace IO {
	namespace Device {
		namespace Actuator {
			namespace ServoMotor {
				namespace B3MSC1170AControlTable {
					constexpr Communicator::Protocols::KondoB3M::Byte
						system_id					= 0x00, 				// eeprom start
						system_baudrate				= 0x01,
						system_position_min			= 0x05,
						system_position_max			= 0x07,
						system_position_center		= 0x09,
						system_mcu_temp_limit		= 0x0B,
						system_mcu_temp_limit_pr	= 0x0D,
						system_motor_temp_limit		= 0x0E,
						system_motor_temp_limit_pr	= 0x10,
						system_current_limit		= 0x11,
						system_current_limit_pr		= 0x13,
						system_lockdetect_time		= 0x14,
						system_lockdetect_outrate	= 0x15,
						system_lockdetect_time_pr	= 0x16,
						system_input_voltage_min	= 0x17,
						system_input_voltage_max	= 0x19,
						system_torque_limit			= 0x1B,
						system_deadband_width		= 0x1C,
						system_motor_cw_ratio		= 0x22,
						system_motor_ccw_ratio		= 0x23,
						servo_servo_option			= 0x27,
						servo_servo_mode			= 0x28,
						servo_torque_on				= 0x28,
						servo_run_mode				= 0x29,
						servo_desired_position		= 0x2A,
						servo_current_position		= 0x2C,
						servo_previous_position		= 0x2E,
						servo_desired_velocity		= 0x30,
						servo_current_velocity		= 0x32,
						servo_previous_velocity		= 0x34,
						servo_desired_time			= 0x36,
						servo_running_time			= 0x38,
						servo_working_time			= 0x3A,
						servo_desired_torque		= 0x3C,
						servo_system_clock			= 0x3E,
						servo_sampling_time			= 0x42,
						servo_mcu_temp				= 0x44,
						servo_motor_temp			= 0x46,
						servo_current				= 0x48,
						servo_input_voltage			= 0x4A,
						servo_pwm_duty				= 0x4C,
						servo_pwm_frequency			= 0x4E,
						servo_encoder_value			= 0x50,
						servo_encoder_count			= 0x52,
						servo_hall_ic_state			= 0x56,
						control_control_law			= 0x5C,
						control_gain_presetno		= 0x5C,
						control_type				= 0x5D,
						control_kp0					= 0x5E,
						control_kd0					= 0x62,
						control_ki0					= 0x66,
						control_static_friction0	= 0x6A,
						control_dynamic_friction0	= 0x6C,
						control_kp1					= 0x6E,
						control_kd1					= 0x72,
						control_ki1					= 0x76,
						control_static_friction1	= 0x7A,
						control_dynamic_friction1	= 0x7C,
						control_kp2					= 0x7E,
						control_kd2					= 0x82,
						control_ki2					= 0x86,
						control_static_friction2	= 0x8A,
						control_dynamic_friction2	= 0x8C,
						status_base_addr			= 0x9D,
						status_system				= 0x9E,
						status_motor				= 0x9F,
						status_uart					= 0xA0,
						status_command				= 0xA1,
						config_model_number			= 0xA2,
						config_model_number_voltage_class	= 0xA2,
						config_model_number_version	= 0xA3,
						config_model_number_torque	= 0xA4,
						config_model_number_case	= 0xA5,
						config_model_type			= 0xA6,
						config_model_type_motor		= 0xA8,
						config_model_type_device	= 0xA9,
						config_fw_version			= 0xAA,
						config_fw_build				= 0xAA,
						config_fw_revision			= 0xAB,
						config_fw_minor				= 0xAC,
						config_fw_major				= 0xAD,
						config_enc_offset_center	= 0xAE,
						config_enc_offset			= 0xB0;
				}
			}
		}
	}
}

