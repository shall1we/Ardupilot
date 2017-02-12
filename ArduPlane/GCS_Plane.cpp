#include "GCS_Plane.h"
#include "Plane.h"

#define FOR_EACH_CHAN(fn)               \
    do  {                                       \
        for (uint8_t i=0; i<_num_gcs; i++) {     \
            _chan[i].fn;                        \
        }                                       \
    } while (0)

#define FOR_EACH_INITIALIZED_CHAN(fn)           \
    do  {                                       \
        for (uint8_t i=0; i<_num_gcs; i++) {     \
            if (_chan[i].initialised) {         \
                _chan[i].fn;                    \
            }                                   \
        }                                       \
    } while (0)

void GCS_Plane::reset_cli_timeout()
{
    FOR_EACH_CHAN(reset_cli_timeout());
}

void GCS_Plane::send_message(enum ap_message id)
{
    FOR_EACH_INITIALIZED_CHAN(send_message(id));
}

void GCS_Plane::data_stream_send()
{
    FOR_EACH_INITIALIZED_CHAN(data_stream_send());
}

void GCS_Plane::update(void)
{
    FOR_EACH_INITIALIZED_CHAN(update(_run_cli));
}

#undef FOR_EACH_CHAN
#undef FOR_EACH_INITIALIZED_CHAN

void GCS_Plane::send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<_num_gcs; i++) {
        if (_chan[i].initialised) {
            _chan[i].mission_item_reached_index = mission_index;
            _chan[i].send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}

void GCS_Plane::send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<_num_gcs; i++) {
        if (_chan[i].initialised) {
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, AIRSPEED_AUTOCAL)) {
                plane.airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
            }
        }
    }
}

void GCS_Plane::setup_uarts(AP_SerialManager &serial_manager)
{
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        _chan[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }
}

void GCS_Plane::handle_interactive_setup()
{
    if (plane.g.cli_enabled == 1) {
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        plane.cliSerial->printf("%s\n", msg);
        if (_chan[1].initialised && (_chan[1].get_uart() != NULL)) {
            _chan[1].get_uart()->printf("%s\n", msg);
        }
        if (num_gcs() > 2 && _chan[2].initialised && (_chan[2].get_uart() != NULL)) {
            _chan[2].get_uart()->printf("%s\n", msg);
        }
    }
}
