/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// NOT AVAILABLE IN AP
//#include "drivers/serial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <stdio.h>



#define SMARTAUDIO_SERIAL_OPTIONS   SERIAL_NOT_INVERTED | SERIAL_BIDIR_NOPULL | SERIAL_STOPBITS_2
#define SMARTAUDIO_DEFAULT_BAUD     4900
#define SMARTAUDIO_MIN_BAUD         4800
#define SMARTAUDIO_MAX_BAUD         4950

#define SMARTAUDIO_SYNC_BYTE            0xAA
#define SMARTAUDIO_HEADER_BYTE          0x55
#define SMARTAUDIO_START_CODE           SMARTAUDIO_SYNC_BYTE + SMARTAUDIO_HEADER_BYTE
#define SMARTAUDIO_GET_PITMODE_FREQ     (1 << 14)
#define SMARTAUDIO_SET_PITMODE_FREQ     (1 << 15)
#define SMARTAUDIO_FREQUENCY_MASK       0x3FFF

#define SMARTAUDIO_CMD_GET_SETTINGS     0x03
#define SMARTAUDIO_CMD_SET_POWER        0x05
#define SMARTAUDIO_CMD_SET_CHANNEL      0x07
#define SMARTAUDIO_CMD_SET_FREQUENCY    0x09
#define SMARTAUDIO_CMD_SET_MODE         0x0B

#define SMARTAUDIO_RSP_GET_SETTINGS_V1  SMARTAUDIO_CMD_GET_SETTINGS >> 1
#define SMARTAUDIO_RSP_GET_SETTINGS_V2  (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x08
#define SMARTAUDIO_RSP_GET_SETTINGS_V21 (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x10
#define SMARTAUDIO_RSP_SET_POWER        SMARTAUDIO_CMD_SET_POWER >> 1
#define SMARTAUDIO_RSP_SET_CHANNEL      SMARTAUDIO_CMD_SET_CHANNEL >> 1
#define SMARTAUDIO_RSP_SET_FREQUENCY    SMARTAUDIO_CMD_SET_FREQUENCY >> 1
#define SMARTAUDIO_RSP_SET_MODE         SMARTAUDIO_CMD_SET_MODE >> 1

#define SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel) (band * 8 + (channel))

#define SMARTAUDIO_SPEC_PROTOCOL_v1  0
#define SMARTAUDIO_SPEC_PROTOCOL_v2  1
#define SMARTAUDIO_SPEC_PROTOCOL_v21 2

// POWER LEVELS: 3 protocols, 4 readings for output in mw
const uint16_t POWER_LEVELS[3][4] =
{
//   25      200   500   800  mw
//   14      23     27    29  dbm
    { 7 , 16, 25, 40}, /* Version 1 */
    { 0 , 1 , 2 , 3 }, /* Version 2 */
};

typedef struct smartaudioSettings_s {
    uint8_t version=0;
    uint8_t unlocked=0;
    uint8_t channel=0;
    uint8_t power=0;
    uint16_t frequency=0;
    uint8_t band=0;

    uint16_t pitmodeFrequency=0;
    bool userFrequencyMode=false;     // user is setting freq
    bool pitmodeDisabled=false;
    bool pitmodeInRangeActive=false;
    bool pitmodeOutRangeActive=false;

    bool frequency_updated=false;
    bool channel_updated=false;
    // true when settings are from parsing response.
    void overall_updated(bool value){
        if(value){
            frequency_updated=true;
            }
    }

} smartaudioSettings_t;

typedef struct smartaudioFrameHeader_s {
    //   uint16_t startCode;
    uint8_t syncByte;
    uint8_t headerByte;
    uint8_t command;
    uint8_t length;
} __attribute__((packed)) smartaudioFrameHeader_t;

typedef struct smartaudioCommandOnlyFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t crc;
} __attribute__((packed)) smartaudioCommandOnlyFrame_t;

typedef struct smartaudioU8Frame_s {
    smartaudioFrameHeader_t header;
    uint8_t payload;
    uint8_t crc;
} __attribute__((packed)) smartaudioU8Frame_t;

typedef struct smartaudioU16Frame_s {
    smartaudioFrameHeader_t header;
    uint16_t payload;
    uint8_t crc;
} __attribute__((packed)) smartaudioU16Frame_t;

typedef struct smartaudioU8ResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t payload;
    uint8_t reserved;
    uint8_t crc;
} __attribute__((packed)) smartaudioU8ResponseFrame_t;

typedef struct smartaudioU16ResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint16_t payload;
    uint8_t reserved;
    uint8_t crc;
} __attribute__((packed)) smartaudioU16ResponseFrame_t;

typedef struct smartaudioSettingsResponseFrame_s {
    smartaudioFrameHeader_t header;
    uint8_t channel;
    uint8_t power;
    uint8_t operationMode;
    uint16_t frequency;
    uint8_t crc;
} __attribute__((packed)) smartaudioSettingsResponseFrame_t;

// v 2.1 additions to response frame
//0x0E (current power in dBm) 0x03 (amount of power levels) 0x00(dBm level 1) 0x0E (dBm level 2) 0x14 (dBm level 3) 0x1A (dBm level 4) 0x01(CRC8)

typedef union smartaudioFrame_u {
    smartaudioCommandOnlyFrame_t commandOnlyFrame;
    smartaudioU8Frame_t u8RequestFrame;
    smartaudioU16Frame_t u16RequestFrame;
} __attribute__((packed)) smartaudioFrame_t;

size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame);
size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power);
size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel);
size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency);
size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings);
bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer);
u_int16_t applyBigEndian16(u_int16_t bytes);
// crc8 from betaflight
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {

        // printf("CURRENT_BYTE:%02x",*p);
        crc = crc8_dvb(crc, *p, 0xD5);
        //  printf("CURRENT_CRC_BYTE:%02x",crc);
    }
    return crc;
}