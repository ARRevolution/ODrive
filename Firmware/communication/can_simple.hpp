#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
        MSG_ODRIVE_ESTOP = 0x001,
        MSG_MOVE_TO_POS,
        MSG_SET_POS_SETPOINT,
        MSG_SET_VEL_SETPOINT,
        MSG_SET_TORQUE_SETPOINT,
        MSG_GET_ENCODER_COUNT,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_GET_IQ,
        MSG_GET_VBUS_VOLTAGE,
        MSG_GET_ADC_VOLTAGE,
        MSG_GET_INVERTER_TEMP,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_SET_VEL_LIMIT,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_SENSORLESS_ESTIMATES,
		MSG_SET_PI_GAIN,
        MSG_CLEAR_ERRORS,
        MSG_RESET_CONTROLLER,
        MSG_CONFIG_MAIN,
        MSG_CONFIG_MOTOR_1,
        MSG_CONFIG_MOTOR_2,
        MSG_CONFIG_MOTOR_3,
        MSG_CONFIG_MOTOR_4,        
        MSG_CONFIG_ENCODER_1,
        MSG_CONFIG_ENCODER_2,
        MSG_CONFIG_CONTROL_1,
        MSG_CONFIG_CONTROL_2,
        MSG_CONFIG_CONTROL_3,
        MSG_CONFIG_GLOBAL,
        MSG_SET_PRE_CAL,
        MSG_GET_CAL_RESULT,
        MSG_RESET_POS,
        MSG_ERASE_CONFIG,
        MSG_SAVE_CONFIG,
        MSG_RESET_ODRIVE,
        MSG_ODRIVE_HEARTBEAT
    };

    static void handle_can_message(can_Message_t& msg);
    static void send_heartbeat(Axis* axis);
    static void sample_pot_pos();

   private:
    static void nmt_callback(Axis* axis, can_Message_t& msg);
    static void estop_callback(Axis* axis, can_Message_t& msg);
    static void get_motor_error_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_error_callback(Axis* axis, can_Message_t& msg);
    static void get_controller_error_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_error_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_nodeid_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_count_callback(Axis* axis, can_Message_t& msg);
    static void move_to_pos_callback(Axis* axis, can_Message_t& msg);
    static void set_pos_setpoint_callback(Axis* axis, can_Message_t& msg);
    static void set_vel_setpoint_callback(Axis* axis, can_Message_t& msg);
    static void set_torque_setpoint_callback(Axis* axis, can_Message_t& msg);
    static void set_vel_limit_callback(Axis* axis, can_Message_t& msg);
    static void start_anticogging_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_inertia_callback(Axis* axis, can_Message_t& msg);
    static void get_iq_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_vbus_voltage_callback(Axis* axis, can_Message_t& msg);
    static void set_pi_gain(Axis* axis, can_Message_t& msg);
    static int compare( const void* a, const void* b);
    static void get_adc_voltage_callback(Axis* axis, can_Message_t& msg);
    static void clear_errors_callback(Axis* axis, can_Message_t& msg);
    static void get_inverter_temp_callback(Axis* axis, can_Message_t& msg);
    static void set_config_callback(Axis* axis, can_Message_t& msg);
    static void set_config_motor_1_callback(Axis* axis, can_Message_t& msg);
    static void set_config_motor_2_callback(Axis* axis, can_Message_t& msg);
    static void set_config_motor_3_callback(Axis* axis, can_Message_t& msg);
    static void set_config_motor_4_callback(Axis* axis, can_Message_t& msg);    
    static void set_config_encoder_1_callback(Axis* axis, can_Message_t& msg);
    static void set_config_encoder_2_callback(Axis* axis, can_Message_t& msg);
    static void set_config_control_1_callback(Axis* axis, can_Message_t& msg);
    static void set_config_control_2_callback(Axis* axis, can_Message_t& msg);
    static void set_config_control_3_callback(Axis* axis, can_Message_t& msg);
    static void set_config_global_callback(Axis* axis, can_Message_t& msg);
    static void set_trigger_calibrate_callback(Axis* axis, can_Message_t& msg);
    static void set_pre_cal_callback(Axis* axis, can_Message_t& msg);
    static void get_cal_result_callback(Axis* axis, can_Message_t& msg);
    static void reset_pos_callback(Axis* axis, can_Message_t& msg);
    
    // Utility functions
    static uint8_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    // Fetch a specific signal from the message

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(can_Message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };
};





#endif