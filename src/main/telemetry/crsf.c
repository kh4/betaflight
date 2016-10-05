/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef TELEMETRY

#include "build/version.h"

#if (FC_VERSION_MAJOR == 3) // not a very good way of finding out if this is betaflight or Cleanflight
#define BETAFLIGHT
#else
#define CLEANFLIGHT
#endif

#ifdef CLEANFLIGHT
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#endif

#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/pwm_rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/gps.h"
#include "io/serial.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/gps.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"

#include "rx/crsf.h"

#include "telemetry/telemetry.h"
#include "telemetry/crsf.h"

#ifdef CLEANFLIGHT
#include "fc/fc_serial.h"
#include "sensors/amperage.h"
#else
#include "fc/config.h"
static telemetryConfig_t *telemetryConfig;
#endif

#define TELEMETRY_CRSF_INITIAL_PORT_MODE    MODE_TX
#define CRSF_CYCLETIME_US                   100000

static serialPort_t *serialPort;
static serialPortConfig_t *serialPortConfig;
static portSharing_e portSharing;
static bool crsfEnabled;
static uint8_t crsf_crc;
static uint8_t crsfPayload[CRSF_MAX_MESSAGE_SIZE];

static void crsf_initialise_packet(sbuf_t *dst)
{
    crsf_crc = 0;
    dst->ptr = crsfPayload;
    dst->end = ARRAYEND(crsfPayload);

    sbufWriteU8(dst, CRSF_RECEIVER_ADDRESS);
}

static void crsf_serialise_8(sbuf_t *dst, uint8_t v)
{
    sbufWriteU8(dst, v);
    crsf_crc = crc8_dvb_s2(crsf_crc, v);
}

static void crsf_serialise_16(sbuf_t *dst, uint16_t v)
{
    // Use BigEndian format
    crsf_serialise_8(dst,  (v >> 8));
    crsf_serialise_8(dst, (uint8_t)v);
}

static void crsf_serialise_32(sbuf_t *dst, uint32_t v)
{
    // Use BigEndian format
    crsf_serialise_8(dst, (v >> 24));
    crsf_serialise_8(dst, (v >> 16));
    crsf_serialise_8(dst, (v >> 8));
    crsf_serialise_8(dst, (uint8_t)v);
}

static void crsf_finalise(sbuf_t *dst)
{
    sbufWriteU8(dst, crsf_crc);
    sbufSwitchToReader(dst, crsfPayload);
    serialWriteBuf(serialPort, sbufPtr(dst), sbufBytesRemaining(dst));
}

static int crsf_finalise_buf(sbuf_t *dst, uint8_t *frame)
{
    sbufWriteU8(dst, crsf_crc);
    sbufSwitchToReader(dst, crsfPayload);
    const int frameSize = sbufBytesRemaining(dst);
    for (int ii = 0; sbufBytesRemaining(dst); ++ii) {
        frame[ii] = sbufReadU8(dst);
    }
    return frameSize;
}
/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t)
*/

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 100 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­ 1000m offset )
uint8_t     Satellites in use ( counter )
*/
void crsf_frame_gps(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsf_serialise_8(dst, CRSF_FRAMETYPE_GPS);
    crsf_serialise_32(dst, GPS_coord[LAT]); // CRSF and betaflight use same units for degrees
    crsf_serialise_32(dst, GPS_coord[LON]);
    crsf_serialise_16(dst, GPS_speed * 36); // GPS_speed is in 0.1m/s
    crsf_serialise_16(dst, GPS_ground_course * 10); // GPS_ground_course is degrees * 10
    //Send real GPS altitude only if it's reliable (there's a GPS fix)
    const uint16_t altitude = (STATE(GPS_FIX) ? GPS_altitude : 0) + 1000;
    crsf_serialise_16(dst, altitude);
    crsf_serialise_8(dst, GPS_numSat);
}

/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Capacity ( mAh )
uint8_t     Battery remaining ( percent )
*/
void crsf_frame_battery_sensor(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsf_serialise_8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR);
    crsf_serialise_16(dst, vbat); // vbat is in units of 0.1V
#ifdef CLEANFLIGHT
    const amperageMeter_t *amperageMeter = getAmperageMeter(batteryConfig()->amperageMeterSource);
    const int16_t amperage = constrain(amperageMeter->amperage, -0x8000, 0x7FFF) / 10; // send amperage in 0.01 A steps, range is -320A to 320A
    crsf_serialise_16(dst, amperage); // amperage is in units of 0.1A
    const uint32_t batteryCapacity = batteryConfig()->batteryCapacity;
    const uint8_t batteryRemainingPercentage = batteryCapacityRemainingPercentage();
#else
    crsf_serialise_16(dst, amperage / 10);
    const uint32_t batteryCapacity = batteryConfig->batteryCapacity;
    const uint8_t batteryRemainingPercentage = calculateBatteryCapacityRemainingPercentage();
#endif
    crsf_serialise_8(dst, (batteryCapacity >> 16));
    crsf_serialise_8(dst, (batteryCapacity >> 8));
    crsf_serialise_8(dst, (uint8_t)batteryCapacity);

    crsf_serialise_8(dst, batteryRemainingPercentage);
}

typedef enum {
    CRSF_ACTIVE_ANTENNA1 = 0,
    CRSF_ACTIVE_ANTENNA2 = 1,
} crsfActiveAntenna_e;

typedef enum {
    CRSF_RF_MODE_4_HZ = 0,
    CRSF_RF_MODE_50_HZ = 1,
    CRSF_RF_MODE_150_HZ = 2,
} crsrRfMode_e;

typedef enum {
    CRSF_RF_POWER_0_mW = 0,
    CRSF_RF_POWER_10_mW = 1,
    CRSF_RF_POWER_25_mW = 2,
    CRSF_RF_POWER_100_mW = 3,
    CRSF_RF_POWER_500_mW = 4,
    CRSF_RF_POWER_1000_mW = 5,
    CRSF_RF_POWER_2000_mW = 6,
} crsrRfPower_e;

/*
0x14 Link statistics
Uplink is the connection from the ground to the UAV and downlink the opposite direction.
Payload:
uint8_t     UplinkRSSI Ant.1(dBm*­1)
uint8_t     UplinkRSSI Ant.2(dBm*­1)
uint8_t     Uplink Package success rate / Link quality ( % )
int8_t      Uplink SNR ( db )
uint8_t     Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
uint8_t     RF Mode ( enum 4fps = 0 , 50fps, 150hz)
uint8_t     Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
uint8_t     Downlink RSSI ( dBm * ­-1 )
uint8_t     Downlink package success rate / Link quality ( % )
int8_t      Downlink SNR ( db )
*/

void crsf_frame_link_statistics(sbuf_t *dst)
{
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsf_serialise_8(dst, CRSF_FRAMETYPE_LINK_STATISTICS);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
    crsf_serialise_8(dst, 0);
}

/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/

#define DECIDEGREES_TO_RADIANS10000(angle) (1000.0f * (angle) * RAD)

void crsf_frame_attitude(sbuf_t *dst)
{
     sbufWriteU8(dst, CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC);
     crsf_serialise_8(dst, CRSF_FRAMETYPE_ATTITUDE);
     crsf_serialise_16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.pitch));
     crsf_serialise_16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.roll));
     crsf_serialise_16(dst, DECIDEGREES_TO_RADIANS10000(attitude.values.yaw));
}

/*
0x21 Flight mode text based
Payload:
char[]      Flight mode ( Null­terminated string )
*/
void crsf_frame_flight_mode(sbuf_t *dst)
{
    // just do Angle for the moment as a placeholder
    const char *flightMode = "Angle";
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst, strlen(flightMode) + 1 + CRSF_FRAME_LENGTH_TYPE_CRC);
    crsf_serialise_8(dst, CRSF_FRAMETYPE_FLIGHT_MODE);
    const int len = strlen(flightMode);
    for (int ii = 0; ii< len; ++ii) {
        crsf_serialise_8(dst, flightMode[ii]);
    }
    crsf_serialise_8(dst, 0);
}

#define BV(x)  (1 << (x)) // bit value

// schedule array to decide how often each type of frame is sent
#define CRSF_SCHEDULE_COUNT     5
static uint8_t crsf_schedule[CRSF_SCHEDULE_COUNT] = {
    BV(CRSF_FRAME_ATTITUDE),
    BV(CRSF_FRAME_BATTERY_SENSOR),
    BV(CRSF_FRAME_LINK_STATISTICS),
    BV(CRSF_FRAME_FLIGHT_MODE),
    BV(CRSF_FRAME_GPS),
};

static void process_crsf(void)
{
    static uint8_t crsf_schedule_index = 0;
    const uint8_t current_schedule = crsf_schedule[crsf_schedule_index];

    sbuf_t crsfPayloadBuf;
    sbuf_t *dst = &crsfPayloadBuf;

    if (current_schedule & BV(CRSF_FRAME_ATTITUDE)) {
        crsf_initialise_packet(dst);
        crsf_frame_attitude(dst);
        crsf_finalise(dst);
    }
    if (current_schedule & BV(CRSF_FRAME_BATTERY_SENSOR)) {
        crsf_initialise_packet(dst);
        crsf_frame_battery_sensor(dst);
        crsf_finalise(dst);
    }
    if (current_schedule & BV(CRSF_FRAME_LINK_STATISTICS)) {
        crsf_initialise_packet(dst);
        crsf_frame_link_statistics(dst);
        crsf_finalise(dst);
    }
    if (current_schedule & BV(CRSF_FRAME_FLIGHT_MODE)) {
        crsf_initialise_packet(dst);
        crsf_frame_flight_mode(dst);
        crsf_finalise(dst);
    }
#ifdef GPS
    if (current_schedule & BV(CRSF_FRAME_GPS)) {
        crsf_initialise_packet(dst);
        crsf_frame_gps(dst);
        crsf_finalise(dst);
    }
#endif
    crsf_schedule_index = (crsf_schedule_index + 1) % CRSF_SCHEDULE_COUNT;
}

void handleCrsfTelemetry(uint32_t currentTime)
{
    static uint32_t crsf_lastCycleTime;
    if (!crsfEnabled) {
        return;
    }
    if (!serialPort) {
        return;
    }
    if ((currentTime - crsf_lastCycleTime) >= CRSF_CYCLETIME_US) {
        process_crsf();
        crsf_lastCycleTime = currentTime;
    }
}

void freeCrsfTelemetryPort(void)
{
    closeSerialPort(serialPort);
    serialPort = NULL;
    crsfEnabled = false;
}

#ifdef CLEANFLIGHT
void initCrsfTelemetry(void)
{
    serialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_CRSF);
    portSharing = determinePortSharing(serialPortConfig, FUNCTION_TELEMETRY_CRSF);
}
#else
void initCrsfTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    serialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_CRSF);
    portSharing = determinePortSharing(serialPortConfig, FUNCTION_TELEMETRY_CRSF);
}
#endif

void configureCrsfTelemetryPort(void)
{
    if (!serialPortConfig) {
        return;
    }
#ifdef CLEANFLIGHT
    baudRate_e baudRateIndex = serialPortConfig->baudRates[BAUDRATE_TELEMETRY];
#else
    baudRate_e baudRateIndex = serialPortConfig->telemetry_baudrateIndex;
#endif
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_400000;
    }
    serialPort = openSerialPort(serialPortConfig->identifier, FUNCTION_TELEMETRY_CRSF, NULL, baudRates[baudRateIndex], TELEMETRY_CRSF_INITIAL_PORT_MODE, SERIAL_INVERTED);
    if (!serialPort) {
        return;
    }
    crsfEnabled = true;
}

bool checkCrsfTelemetryState(void)
{
    const bool newTelemetryEnabled = telemetryDetermineEnabledState(portSharing);
    if (newTelemetryEnabled == crsfEnabled) {
        return false;
    }
    if (newTelemetryEnabled) {
        configureCrsfTelemetryPort();
    } else {
        freeCrsfTelemetryPort();
    }
    return true;
}

int getCrsfFrame(uint8_t *frame, crsf_frame_e frameType)
{
    sbuf_t crsfPayloadBuf;
    sbuf_t *sbuf = &crsfPayloadBuf;

    crsf_initialise_packet(sbuf);
    switch (frameType) {
    default:
    case CRSF_FRAME_ATTITUDE:
        crsf_frame_attitude(sbuf);
        break;
    case CRSF_FRAME_BATTERY_SENSOR:
        crsf_frame_battery_sensor(sbuf);
        break;
    case CRSF_FRAME_LINK_STATISTICS:
        crsf_frame_link_statistics(sbuf);
        break;
    case CRSF_FRAME_FLIGHT_MODE:
        crsf_frame_flight_mode(sbuf);
        break;
#if defined(GPS)
    case CRSF_FRAME_GPS:
        crsf_frame_gps(sbuf);
        break;
#endif
    }
    const int frameSize = crsf_finalise_buf(sbuf, frame);
    return frameSize;
}
#endif
