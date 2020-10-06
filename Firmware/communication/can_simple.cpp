
#include "can_simple.hpp"
#include <odrive_main.h>

#include <cstring>

static const uint8_t NUM_NODE_ID_BITS = 4;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

static const uint8_t num_adc_samples = 100;
float adc_samples[num_adc_samples] = {0.0};
uint8_t adc_sample_count = 0; 

void CANSimple::handle_can_message(can_Message_t& msg) {
    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking to fix the syntax.
    //
    // auto func = callback_map.find(msg.id);
    // if(func != callback_map.end()){
    //     func->second(msg);
    // }

    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);
    uint32_t cmd = get_cmd_id(msg.id);

    Axis* axis = nullptr;

    bool validAxis = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if ((axes[i].config_.can_node_id == nodeID) && (axes[i].config_.can_node_id_extended == msg.isExt)) {
            axis = &axes[i];
            if (!validAxis) {
                validAxis = true;
            } else {
                // Duplicate can IDs, don't assign to any axis
                odCAN->set_error(ODriveCAN::ERROR_DUPLICATE_CAN_IDS);
                validAxis = false;
                break;
            }
        }
    }

    if (validAxis) {
        axis->watchdog_feed();
        switch (cmd) {
            case MSG_CO_NMT_CTRL:
                break;
            case MSG_CO_HEARTBEAT_CMD:
                break;
            case MSG_ODRIVE_HEARTBEAT:
                // We don't currently do anything to respond to ODrive heartbeat messages
                break;
            case MSG_ODRIVE_ESTOP:
                estop_callback(axis, msg);
                break;
            case MSG_GET_MOTOR_ERROR:
                get_motor_error_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ERROR:
                get_encoder_error_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ERROR:
                get_sensorless_error_callback(axis, msg);
                break;
            case MSG_SET_AXIS_NODE_ID:
                set_axis_nodeid_callback(axis, msg);
                break;
            case MSG_SET_AXIS_REQUESTED_STATE:
                set_axis_requested_state_callback(axis, msg);
                break;
            case MSG_SET_AXIS_STARTUP_CONFIG:
                set_axis_startup_config_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ESTIMATES:
                get_encoder_estimates_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_COUNT:
                get_encoder_count_callback(axis, msg);
                break;
            case MSG_MOVE_TO_POS:
                move_to_pos_callback(axis, msg);
                break;
            case MSG_SET_POS_SETPOINT:
                set_pos_setpoint_callback(axis, msg);
                break;
            case MSG_SET_VEL_SETPOINT:
                set_vel_setpoint_callback(axis, msg);
                break;
            case MSG_SET_TORQUE_SETPOINT:
                set_torque_setpoint_callback(axis, msg);
                break;
            case MSG_SET_VEL_LIMIT:
                set_vel_limit_callback(axis, msg);
                break;
            case MSG_START_ANTICOGGING:
                start_anticogging_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_INERTIA:
                set_traj_inertia_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_ACCEL_LIMITS:
                set_traj_accel_limits_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_VEL_LIMIT:
                set_traj_vel_limit_callback(axis, msg);
                break;
            case MSG_GET_IQ:
                get_iq_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ESTIMATES:
                get_sensorless_estimates_callback(axis, msg);
                break;
            case MSG_RESET_ODRIVE:
                NVIC_SystemReset();
                break;
            case MSG_SAVE_CONFIG:
                odrv.save_configuration();
                break;
            case MSG_ERASE_CONFIG:
                odrv.erase_configuration();
                break;
            case MSG_GET_VBUS_VOLTAGE:
                get_vbus_voltage_callback(axis, msg);
                break;
			case MSG_SET_PI_GAIN:
				set_pi_gain(axis, msg);
				break;	
            case MSG_CLEAR_ERRORS:
                clear_errors_callback(axis, msg);
                break;
            case MSG_GET_ADC_VOLTAGE:
                get_adc_voltage_callback(axis, msg);
                break;
            case MSG_RESET_CONTROLLER:
                axis->controller_.reset();
                break;
            case MSG_GET_INVERTER_TEMP:
                get_inverter_temp_callback(axis, msg);
                break;
            case MSG_CONFIG_MAIN:
                set_config_callback(axis, msg);
                break;
            case MSG_CONFIG_MOTOR_1:
                set_config_motor_1_callback(axis, msg);
                break;
            case MSG_CONFIG_MOTOR_2:
                set_config_motor_2_callback(axis, msg);
                break;
            case MSG_CONFIG_MOTOR_3:
                set_config_motor_3_callback(axis, msg);
                break;
            case MSG_CONFIG_MOTOR_4:
                set_config_motor_4_callback(axis, msg);
                break;                
            case MSG_CONFIG_ENCODER_1:
                set_config_encoder_1_callback(axis, msg);
                break;
            case MSG_CONFIG_ENCODER_2:
                set_config_encoder_2_callback(axis, msg);
                break;
            case MSG_CONFIG_CONTROL_1:
                set_config_control_1_callback(axis, msg);
                break;
            case MSG_CONFIG_CONTROL_2:
                set_config_control_2_callback(axis, msg);
                break;
            case MSG_CONFIG_CONTROL_3:
                set_config_control_3_callback(axis, msg);
                break;
            case MSG_CONFIG_GLOBAL:
                set_config_global_callback(axis, msg);
                break;
            case MSG_SET_PRE_CAL:
                set_pre_cal_callback(axis, msg);
                break;
            case MSG_GET_CAL_RESULT:
                get_cal_result_callback(axis, msg);
                break;
            case MSG_RESET_POS:
                reset_pos_callback(axis, msg);
                break;
            default:
                break;
        }
    }

    // Do ADC sample store
    adc_samples[adc_sample_count++] = odrv.get_adc_voltage(5); //get_gpio_port_by_pin(5), get_gpio_pin_by_pin(5));
    if (adc_sample_count >= num_adc_samples)
        adc_sample_count = 0;
}

void CANSimple::nmt_callback(Axis* axis, can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis* axis, can_Message_t& msg) {
    axis->error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

void CANSimple::get_motor_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_MOTOR_ERROR << NUM_NODE_ID_BITS; 
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->motor_.error_;
        txmsg.buf[1] = axis->motor_.error_ >> 8;
        txmsg.buf[2] = axis->motor_.error_ >> 16;
        txmsg.buf[3] = axis->motor_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_ENCODER_ERROR << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.error_;
        txmsg.buf[1] = axis->encoder_.error_ >> 8;
        txmsg.buf[2] = axis->encoder_.error_ >> 16;
        txmsg.buf[3] = axis->encoder_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_SENSORLESS_ERROR << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->sensorless_estimator_.error_;
        txmsg.buf[1] = axis->sensorless_estimator_.error_ >> 8;
        txmsg.buf[2] = axis->sensorless_estimator_.error_ >> 16;
        txmsg.buf[3] = axis->sensorless_estimator_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_axis_nodeid_callback(Axis* axis, can_Message_t& msg) {
    axis->config_.can_node_id = msg.buf[0] & 0x3F;  // Node ID bitmask
}

void CANSimple::set_axis_requested_state_callback(Axis* axis, can_Message_t& msg) {
    axis->requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int16_t>(msg, 0, 16, true, 1, 0));
}
void CANSimple::set_axis_startup_config_callback(Axis* axis, can_Message_t& msg) {
    // Not Implemented
}

void CANSimple::get_encoder_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_ENCODER_ESTIMATES << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->encoder_.pos_estimate_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->encoder_.pos_estimate_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->encoder_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->encoder_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_SENSORLESS_ESTIMATES << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->sensorless_estimator_.pll_pos_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.pll_pos_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->sensorless_estimator_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_count_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_ENCODER_COUNT << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.shadow_count_;
        txmsg.buf[1] = axis->encoder_.shadow_count_ >> 8;
        txmsg.buf[2] = axis->encoder_.shadow_count_ >> 16;
        txmsg.buf[3] = axis->encoder_.shadow_count_ >> 24;

        txmsg.buf[4] = axis->encoder_.count_in_cpr_;
        txmsg.buf[5] = axis->encoder_.count_in_cpr_ >> 8;
        txmsg.buf[6] = axis->encoder_.count_in_cpr_ >> 16;
        txmsg.buf[7] = axis->encoder_.count_in_cpr_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::move_to_pos_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.move_to_pos(can_getSignal<int32_t>(msg, 0, 32, true, 1, 0));
}

void CANSimple::set_pos_setpoint_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_pos_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis->controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis->controller_.input_pos_updated();
}

void CANSimple::set_vel_setpoint_callback(Axis* axis, can_Message_t& msg) { // 0.1f??
    axis->controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_torque_setpoint_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_vel_limit_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true, 1, 0);
}

void CANSimple::start_anticogging_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis->trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::get_iq_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_IQ << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof axis->motor_.current_control_.Iq_setpoint == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_setpoint, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->motor_.current_control_.Iq_measured);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_measured, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_vbus_voltage_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;

        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_VBUS_VOLTAGE << NUM_NODE_ID_BITS;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof vbus_voltage == sizeof floatBytes);
        std::memcpy(&floatBytes, &vbus_voltage, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_pi_gain(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::clear_errors_callback(Axis* axis, can_Message_t& msg) {
    axis->clear_errors();
}

void CANSimple::send_heartbeat(Axis* axis) {
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id;
    txmsg.id += MSG_ODRIVE_HEARTBEAT << NUM_NODE_ID_BITS;  // heartbeat ID
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;

    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = axis->error_;
    txmsg.buf[1] = axis->error_ >> 8;
    txmsg.buf[2] = axis->error_ >> 16;
    txmsg.buf[3] = axis->error_ >> 24;

    // Current state of axis in 2nd 32-bit value
    txmsg.buf[4] = axis->current_state_;
    txmsg.buf[5] = axis->current_state_ >> 8;
    txmsg.buf[6] = axis->current_state_ >> 16;
    txmsg.buf[7] = axis->current_state_ >> 24;
    odCAN->write(txmsg);
}

int CANSimple::compare( const void* a, const void* b)
{
     float float_a = * ( (float*) a );
     float float_b = * ( (float*) b );

     if ( float_a == float_b ) return 0;
     else if ( float_a < float_b ) return -1;
     else return 1;
}

void CANSimple::get_adc_voltage_callback(Axis* axis, can_Message_t& msg) {
        can_Message_t txmsg;

        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_ADC_VOLTAGE << NUM_NODE_ID_BITS;
        txmsg.isExt = false;
        txmsg.len = 8;

		//uint32_t gpio_num_in = msg.buf[0] & 0xF;

        //float sorted_adc_samples[20] = {0.0};
        //for (int i = 0; i++; i < 20)
        //    sorted_adc_samples[i] = Encoder::adc_samples[i];
        
        qsort(adc_samples, num_adc_samples, sizeof(float), compare );

        float adc_in_voltage = adc_samples[num_adc_samples/2]; //get_adc_voltage(get_gpio_port_by_pin(gpio_num_in), get_gpio_pin_by_pin(gpio_num_in));

        uint32_t floatBytes;
        static_assert(sizeof adc_in_voltage == sizeof floatBytes);
        std::memcpy(&floatBytes, &adc_in_voltage, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
}

void CANSimple::get_inverter_temp_callback(Axis* axis, can_Message_t& msg)
{
    if (msg.rtr) {
        can_Message_t txmsg;

        txmsg.id = axis->config_.can_node_id;
        txmsg.id += MSG_GET_INVERTER_TEMP << NUM_NODE_ID_BITS;
        txmsg.isExt = false;
        txmsg.len = 8;

        float inverter_temp = axis->fet_thermistor_.temperature_;

        uint32_t floatBytes;
        static_assert(sizeof inverter_temp == sizeof floatBytes);
        std::memcpy(&floatBytes, &inverter_temp, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_config_callback(Axis* axis, can_Message_t& msg)
{
    axis->config_.can_heartbeat_rate_ms = can_getSignal<uint32_t>(msg, 0, 32, true, 1, 0);
}

void CANSimple::set_config_motor_1_callback(Axis* axis, can_Message_t& msg)
{
    axis->motor_.config_.pole_pairs = can_getSignal<int32_t>(msg, 0, 32, true, 1, 0);
    axis->motor_.config_.resistance_calib_max_voltage = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_motor_2_callback(Axis* axis, can_Message_t& msg)
{
    axis->motor_.config_.requested_current_range = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->motor_.config_.current_control_bandwidth = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_motor_3_callback(Axis* axis, can_Message_t& msg)
{
    axis->motor_.config_.calibration_current = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_motor_4_callback(Axis* axis, can_Message_t& msg)
{
    axis->motor_.config_.current_lim_margin = can_getSignal<float>(msg, 0, 32, true, 1, 0);
}

void CANSimple::set_config_encoder_1_callback(Axis* axis, can_Message_t& msg)
{
    axis->encoder_.config_.mode = static_cast<Encoder::Mode>(can_getSignal<int8_t>(msg, 0, 8, true, 1, 0));
    axis->encoder_.config_.ignore_illegal_hall_state = (bool)can_getSignal<int8_t>(msg, 8, 8, true, 1, 0);
}

void CANSimple::set_config_encoder_2_callback(Axis* axis, can_Message_t& msg)
{
    axis->encoder_.config_.cpr = can_getSignal<int32_t>(msg, 0, 32, true, 1, 0);
    axis->encoder_.config_.bandwidth = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_control_1_callback(Axis* axis, can_Message_t& msg)
{
    axis->controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->controller_.config_.vel_limit_tolerance = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_control_2_callback(Axis* axis, can_Message_t& msg)
{
    axis->controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->controller_.config_.vel_gain = can_getSignal<float>(msg, 32, 32, true, 1, 0);
}

void CANSimple::set_config_control_3_callback(Axis* axis, can_Message_t& msg)
{
    axis->controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 0, 32, true, 1, 0);
    axis->controller_.config_.control_mode = static_cast<Controller::ControlMode>(can_getSignal<int8_t>(msg, 32, 8, true, 1, 0));
}

void CANSimple::set_config_global_callback(Axis* axis, can_Message_t& msg)
{
    odrv.config_.brake_resistance = can_getSignal<float>(msg, 0, 32, true, 1, 0);
}

void CANSimple::set_pre_cal_callback(Axis* axis, can_Message_t& msg)
{
    // Set both precalibrated
    axis->motor_.config_.pre_calibrated = (bool)can_getSignal<int8_t>(msg, 0, 8, true, 1, 0);
    axis->encoder_.config_.pre_calibrated = (bool)can_getSignal<int8_t>(msg, 8, 8, true, 1, 0);
    axis->config_.startup_closed_loop_control = static_cast<bool>(can_getSignal<int8_t>(msg, 16, 8, true, 1, 0));

    // Save config - if calibration failed then pre_cal can't be set so makes no difference.
    odrv.save_configuration();
}

void CANSimple::get_cal_result_callback(Axis* axis, can_Message_t& msg)
{
    // Return Calibration info
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id;
    txmsg.id += MSG_GET_CAL_RESULT << NUM_NODE_ID_BITS;
    txmsg.isExt = false;
    txmsg.len = 8;

    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = axis->error_;
    txmsg.buf[1] = axis->error_ >> 8;
    txmsg.buf[2] = axis->error_ >> 16;
    txmsg.buf[3] = axis->error_ >> 24;

    txmsg.buf[4] = (axis->motor_.is_calibrated_ & 0x1) | ((axis->encoder_.is_ready_ & 0x1) << 1);
    txmsg.buf[5] = 0;
    txmsg.buf[6] = 0;
    txmsg.buf[7] = 0;
    
    odCAN->write(txmsg);
}

void CANSimple::reset_pos_callback(Axis* axis, can_Message_t& msg)
{
    axis->encoder_.set_linear_count(0);
    axis->controller_.input_pos_ = 0;
    axis->controller_.input_vel_ = 0;
    axis->controller_.input_torque_ = 0;
    axis->controller_.input_pos_updated();
}

uint8_t CANSimple::get_node_id(uint32_t msgID) {
    //return ((msgID >> NUM_CMD_ID_BITS) & 0x03F);  // Upper 6 bits
    return (msgID & 0xF);  // Bottom 4 bits
}

uint8_t CANSimple::get_cmd_id(uint32_t msgID) {
    //return (msgID & 0x07F);  // Bottom 7 bits
    return ((msgID >> NUM_NODE_ID_BITS) & 0x7F);  // Upper 7 bits
}