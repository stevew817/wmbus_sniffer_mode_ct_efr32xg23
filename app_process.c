/***************************************************************************//**
 * @file
 * @brief app_process.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "rail.h"
#include "sl_component_catalog.h"
#include "app_process.h"
#include "sl_simple_led_instances.h"
#include "app_log.h"
#include "sl_wmbus_support.h"
#include "em_emu.h"

#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define RAIL_FIFO_SIZE (512U)

#ifndef WMBUS_SNIFFER_PRINT_RAW
#define WMBUS_SNIFFER_PRINT_RAW 1
#endif

#ifndef WMBUS_SNIFFER_PRINT_VERBOSE
#define WMBUS_SNIFFER_PRINT_VERBOSE 1
#endif

#ifndef WMBUS_LOG_3OF6_ERROR
#define WMBUS_LOG_3OF6_ERROR 1
#endif

typedef enum {
  MODE_T_FRAME_A,
  MODE_C_FRAME_A,
  MODE_C_FRAME_B,
  MODE_INVALID
} wmbus_packet_type_t;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
static void print_blocks(const uint8_t *buffer, uint16_t length);
static void print_rx_packets(RAIL_Handle_t rail_handle);
static bool decode3of6_buf(uint8_t* buf, size_t len);
static size_t packet_len_frame_a(uint8_t lfield);
static size_t packet_len_frame_b(uint8_t lfield);
static void RAILCb_Timer(RAIL_Handle_t railHandle);
static void print_raw_packet(RAIL_RxPacketDetails_t* details, wmbus_packet_type_t type, uint8_t* packet, size_t plen);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
static volatile state_t state = S_IDLE;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t current_rail_err = 0;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

/// Receive FIFO
static uint8_t rx_fifo[RAIL_FIFO_SIZE];
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
void set_next_state(state_t next_state)
{
  state = next_state;
}

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(RAIL_Handle_t rail_handle)
{
  (void) rail_handle;

  switch (state) {
    case S_PACKET_RECEIVED:
      print_rx_packets(rail_handle);
      state = S_IDLE;
#if defined(SL_CATALOG_KERNEL_PRESENT)
      app_task_notify();
#endif
      break;
    case S_RX_PACKET_ERROR:
      // Handle Rx error
      app_log_error("Radio RX Error occurred\nEvents: %lld\n", current_rail_err);
      state = S_IDLE;
#if defined(SL_CATALOG_KERNEL_PRESENT)
      app_task_notify();
#endif
      break;
    case S_CALIBRATION_ERROR:
      app_log_warning("Radio Calibration Error occurred\nEvents: %lld\nRAIL_Calibrate() result:%d\n",
                      current_rail_err,
                      calibration_status);
      state = S_IDLE;
#if defined(SL_CATALOG_KERNEL_PRESENT)
      app_task_notify();
#endif
      break;
    case S_IDLE:
      break;
    default:
      break;
  }

  ///////////////////////////////////////////////////////////////////////////
  // Put your application code here!                                       //
  // This is called infinitely.                                            //
  // Do not call blocking functions from here!                             //
  ///////////////////////////////////////////////////////////////////////////
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_RX_SYNC1_DETECT) {
    // trigger a timer for later on setting the full packet length manually.
    // Initially setting to zero (= unlimited packet).
    // need at least 24 chips @ 100kcps for being able to figure out the packet length
    // => 240 us. Set timer to 260 to allow for processing.
    RAIL_SetFixedLength(rail_handle, 0);
    RAIL_SetTimer(rail_handle, 260, RAIL_TIME_DELAY, RAILCb_Timer);
  }

  if (events & RAIL_EVENTS_RX_COMPLETION) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      // Keep the packet in the radio buffer, download it later at the state machine
      RAIL_HoldRxPacket(rail_handle);
      state = S_PACKET_RECEIVED;
    } else {
      // Handle Rx error
      current_rail_err = (events & RAIL_EVENTS_RX_COMPLETION);
      state = S_RX_PACKET_ERROR;
    }
  }

  // Perform all calibrations when needed
  if (events & RAIL_EVENT_CAL_NEEDED) {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR) {
      current_rail_err = (events & RAIL_EVENT_CAL_NEEDED);
      state = S_CALIBRATION_ERROR;
    }
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

static void print_blocks(const uint8_t *buffer, uint16_t length)
{
  for (uint8_t i = 0; i < length; i++) {
    if (i % 16 == 0) {
      app_log_info("[0x%02X ", buffer[i]);
    } else if (i % 16 == 8) {
      app_log_info("| 0x%02X ", buffer[i]);
    } else if (i % 16 == 15 || i == length - 1) {
      app_log_info("0x%02X]\n", buffer[i]);
    } else {
      app_log_info("0x%02X ", buffer[i]);
    }
  }
}

// Printed format:
// RX:timestamp:rssi:mode:frame:packetbytes\n
// timestamp in human-readable microseconds
// rssi in human-readable dbm
// mode is either 'C' or 'T'
// frame is either 'A' or 'B'
// packet bytes in ASCII
#if WMBUS_SNIFFER_PRINT_RAW
static void print_raw_packet(RAIL_RxPacketDetails_t* details, wmbus_packet_type_t type, uint8_t* packet, size_t plen)
{
  app_log_info("RX:%lu:%d:", details->timeReceived.packetTime, details->rssi);
  switch (type) {
    case MODE_T_FRAME_A:
      app_log_info("T:A:");
      break;
    case MODE_C_FRAME_A:
      app_log_info("C:A:");
      break;
    case MODE_C_FRAME_B:
      app_log_info("C:B:");
      break;
    default:
      app_log_info("X:X:");
      break;
  }
  for (size_t i = 0; i< plen; i++) {
      app_log_info("%02X", packet[i]);
  }
  app_log_info("\n");
}
#endif

static void print_rx_packets(RAIL_Handle_t rail_handle)
{
  RAIL_RxPacketInfo_t packet_info;
  RAIL_Status_t rail_status;
  RAIL_RxPacketDetails_t packet_details;
  RAIL_RxPacketHandle_t rx_packet_handle;
  wmbus_packet_type_t packet_type = MODE_INVALID;
  uint8_t* packet_start = rx_fifo;
  size_t packet_length = 0;

  rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);

  while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
    packet_details.timeReceived.totalPacketBytes = packet_info.packetBytes;
    packet_details.timeReceived.timePosition = RAIL_PACKET_TIME_AT_SYNC_END;
    RAIL_GetRxPacketDetails(rail_handle, rx_packet_handle, &packet_details);

    RAIL_CopyRxPacket(rx_fifo, &packet_info);
    rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
    if (rail_status != RAIL_STATUS_NO_ERROR) {
      app_log_warning("RAIL_ReleaseRxPacket() result:%d", rail_status);
    }

    // Do mode decoding if applicable and strip sync bytes
    if (rx_fifo[0] == 0x54) {
      // Mode C (packet starting with 0x54 is not valid 3of6)
      if (rx_fifo[1] == 0xCD) {
        packet_type = MODE_C_FRAME_A;
      } else if (rx_fifo[1] == 0x3D) {
        packet_type = MODE_C_FRAME_B;
      } else {
        app_log_info("Unknown mode C frame format sync byte 0x%02X received\n", rx_fifo[1]);
        packet_type = MODE_INVALID;
        goto next;
      }

      if (packet_type != MODE_INVALID) {
        packet_start = rx_fifo + 2;
        packet_length = packet_details.timeReceived.totalPacketBytes - 2;
      }
    } else {
      // Mode T, attempt 3of6
      if (decode3of6_buf(rx_fifo, packet_details.timeReceived.totalPacketBytes)) {
        packet_start = rx_fifo;
        packet_length = packet_len_frame_a(rx_fifo[0]);
        packet_type = MODE_T_FRAME_A;
      } else {
        // Packet contains invalid 3of6 codes
        packet_type = MODE_INVALID;
        app_log_info("Invalid 3of6 packet received (RSSI %d with starting bytes %02X%02X%02X%02X0)\n", packet_details.rssi, rx_fifo[0], rx_fifo[1], rx_fifo[2], rx_fifo[3] );
        goto next;
      }
    }

    // Throw away invalid packets
    if (packet_info.packetBytes <= 12) {
      app_log_info("Packet too short (%d bytes starting with %02X%02X%02X%02X)\n", packet_info.packetBytes, rx_fifo[0], rx_fifo[1], rx_fifo[2], rx_fifo[3]);
      goto next;
    }

#if WMBUS_SNIFFER_PRINT_RAW
    // Print raw packet
    print_raw_packet(&packet_details, packet_type, packet_start, packet_length);
#endif

#if WMBUS_SNIFFER_PRINT_VERBOSE
    // Strip CRC bytes (ideally check them, but that's TODO)
    if (packet_type == MODE_T_FRAME_A || packet_type == MODE_C_FRAME_A) {
      // Strip CRC over DLL
      memmove(&packet_start[10], &packet_start[12], packet_length - 10 - 2);
      packet_length -= 2;

      // Strip CRCs over data blocks
      size_t num_blocks = (packet_length - 10) / 18;
      size_t extra_bytes = (packet_length - 10) % 18;

      for (size_t i = 1; i <= num_blocks; i++) {
        memmove(&packet_start[10 + (i * 16)], &packet_start[10 + (i * 16) + 2], packet_length - 10 - 2 - ((i-1) * 16));
        packet_length -= 2;
      }

      if (extra_bytes > 0) {
        packet_length -= 2;
      }
    }

    if (packet_type == MODE_C_FRAME_B) {
      if (packet_length <= 128) {
        packet_length -= 2;
      } else {
        memmove(&packet_start[126], &packet_start[128], packet_length - 128 - 2);
        packet_length -= 4;
      }
    }

    // Print more detailed info
    const WMBUS_dll_header_t *dllHeader = (WMBUS_dll_header_t*)packet_start;
    const WMBUS_stl_header_t *stlHeader = (WMBUS_stl_header_t*)(packet_start + sizeof(WMBUS_dll_header_t));

    char mField[3];
    WMBUSframe_MField2Chars(dllHeader->address.detailed.manufacturer, mField);
    app_log_info("RX:[Time:%lu]\n", packet_details.timeReceived.packetTime);
    app_log_info("Block-1:[L:%d,C:0x%02X,M:%c%c%c,ID:%08X,Version:0x%02X,devType:0x%02X]\n",
                 dllHeader->lField,
                 dllHeader->cField.raw,
                 mField[0], mField[1], mField[2],
                 (unsigned int)dllHeader->address.detailed.id,
                 (unsigned int)dllHeader->address.detailed.version,
                 (unsigned int)dllHeader->address.detailed.deviceType);
    if (stlHeader->ciField == WMBUS_CI_EN13757_3_APPLICATION_SHORT) {
      uint8_t *payload_start = packet_start + sizeof(WMBUS_dll_header_t) + sizeof(WMBUS_stl_header_t);
      uint16_t payload_len = dllHeader->lField - sizeof(WMBUS_dll_header_t) - sizeof(WMBUS_stl_header_t) + 1;
      app_log_info("AppHeader:[CI:0x%02X,AccessNr:%d,Status:0x%02X,encMode:%d,Accessibility:%02X,encBlocks:%d,sync:%d]\n",
                   stlHeader->ciField,
                   stlHeader->accessNumber,
                   stlHeader->status,
                   stlHeader->confWord.mode_0_5.mode,
                   stlHeader->confWord.mode_0_5.accessibility,
                   stlHeader->confWord.mode_0_5.numOfEncBlocks,
                   stlHeader->confWord.mode_0_5.synchronized);
      if (stlHeader->confWord.mode_0_5.mode == 5) {
        uint8_t iv[16];
        //with long transport layer header, the address from the header should be used
        memcpy(iv, &(dllHeader->address.raw), 8);
        memset(iv + 8, stlHeader->accessNumber, 8);
        WMBUSframe_crypto5decrypt(payload_start, payload_start, iv, payload_len);
      }
      print_blocks(payload_start, payload_len);
    } else if (stlHeader->ciField == WMBUS_CI_EN13757_3_APPLICATION_LONG) {
      const WMBUS_ltl_header_t *ltlHeader = (WMBUS_ltl_header_t*)(packet_start + sizeof(WMBUS_dll_header_t));
      uint8_t *payload_start = rx_fifo + sizeof(WMBUS_dll_header_t) + sizeof(WMBUS_ltl_header_t);
      uint16_t payload_len = dllHeader->lField - sizeof(WMBUS_dll_header_t) - sizeof(WMBUS_ltl_header_t) + 1;
      app_log_info("AppHeader:[CI:0x%02X,AccessNr:%d,Status:0x%02X,encMode:%d,Accessibility:%02X,encBlocks:%d,sync:%d]\n",
                   ltlHeader->ciField,
                   ltlHeader->accessNumber,
                   ltlHeader->status,
                   ltlHeader->confWord.mode_0_5.mode,
                   ltlHeader->confWord.mode_0_5.accessibility,
                   ltlHeader->confWord.mode_0_5.numOfEncBlocks,
                   ltlHeader->confWord.mode_0_5.synchronized);
      if (ltlHeader->confWord.mode_0_5.mode == 5) {
        uint8_t iv[16];
        //with long transport layer header, the address from the header should be used
        memcpy(iv, &(ltlHeader->address.raw), 8);
        memset(iv + 8, ltlHeader->accessNumber, 8);
        WMBUSframe_crypto5decrypt(payload_start, payload_start, iv, payload_len);
      }
      print_blocks(payload_start, payload_len);
    } else {
      print_blocks(packet_start + sizeof(WMBUS_dll_header_t), dllHeader->lField - sizeof(WMBUS_dll_header_t) + 1);
    }
#endif

next:
    sl_led_toggle(&sl_led_led0);

    rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
  }
}

/******************************************************************************
 * EN 13757-4 data conversion helper functions.
 *****************************************************************************/

/* Software decoder for '3of6' according to EN 13757-4.
 * Input: 6 bits, right-aligned
 * Output: 4 bits, right-aligned
 */
static uint8_t decode3of6(uint8_t input) {
  switch(input) {
  case 0x16:
    return 0;
  case 0x0D:
    return 1;
  case 0x0E:
    return 2;
  case 0x0B:
    return 3;
  case 0x1C:
    return 4;
  case 0x19:
    return 5;
  case 0x1A:
    return 6;
  case 0x13:
    return 7;
  case 0x2C:
    return 8;
  case 0x25:
    return 9;
  case 0x26:
    return 10;
  case 0x23:
    return 11;
  case 0x34:
    return 12;
  case 0x31:
    return 13;
  case 0x32:
    return 14;
  case 0x29:
    return 15;
  default:
    return 16;
  }
}

static bool decode3of6_buf(uint8_t* buf, size_t len)
{
  // output length in bytes = input length / 3 * 2
  size_t olen = (len * 2) / 3;

  for(size_t i = 0; i < (olen*2); i++) {
    // process decoding per output nibble
    // location of input related to output
    size_t base = i / 4 * 3;
    uint8_t code;
    if (i % 4 == 0) {
      code = (buf[base] & 0xFC) >> 2;
    } else if (i % 4 == 1) {
      code = (buf[base] & 0x03) << 4;
      code |= (buf[base + 1] & 0xF0) >> 4;
    } else if (i % 4 == 2) {
      code = (buf[base + 1] & 0x0F) << 2;
      code |= (buf[base + 2] & 0xC0) >> 6;
    } else {
      code = (buf[base + 2] & 0x3F) >> 0;
    }

    code = decode3of6(code);
    if (code >= 16) {
#if WMBUS_LOG_3OF6_ERROR
      app_log_info("Error decoding 3of6 at output nibble %d\n", i);
#endif
      return false;
    }
    if (i % 2 == 0) {
      buf[i/2] = (buf[i/2] & 0x0F) | (code << 4);
    } else {
      buf[i/2] = (buf[i/2] & 0xF0) | code;
    }
  }
  return true;
}

static size_t packet_len_frame_a(uint8_t lfield) {
  if (lfield <= 9) {
    // invalid size
    return 0;
  }

  size_t data_bytes = lfield - 9;
  size_t num_blocks = data_bytes / 16;
  size_t extra_bytes = (data_bytes % 16 == 0 ? 0 : ((data_bytes % 16) + 2));

  return 12 + num_blocks * 18 + extra_bytes;
}

static size_t packet_len_frame_b(uint8_t lfield) {
  if (lfield < 12) {
    // invalid size
    return 0;
  }

  if (lfield <= 127) {
    // 1 byte L field
    // value of L = amount of following bytes excluding CRC
    // 2 bytes CRC
    return 1 + lfield + 2;
  } else if (lfield >= 129) {
    // refer to EN 13757-4: max size of first + second block is 128 bytes
    // optional block is (L - 129) bytes + 2 bytes CRC
    return 128 + (lfield - 129) + 2;
  } else {
    return 128;
  }
}

/******************************************************************************
 * RAIL timer callback, called after just enough time to be able to detect the
 * packet format and length.
 *****************************************************************************/
static void RAILCb_Timer(RAIL_Handle_t railHandle) {
  // Get the first bytes from the radio to determine the packet length
  uint8_t pktBuf[3];
  uint8_t read = RAIL_PeekRxPacket(railHandle, RAIL_RX_PACKET_HANDLE_NEWEST, pktBuf, 3, 0);
  if (read < 3) {
    // We'll filter out these packets later
    RAIL_SetFixedLength(railHandle, 6);
  } else {
    if (pktBuf[0] == 0x54) {
      // This is a modeC packet
      if(pktBuf[1] == 0xCD) {
        // Format A
        // Add 2 bytes because of extra sync vs mode T
        uint16_t len = packet_len_frame_a(pktBuf[2]) + 2;
        if (len > 12) {
          // Set packet length for mode C frame A
          RAIL_SetFixedLength(railHandle, len);
        } else {
          // Set packet length short - invalid header
          RAIL_SetFixedLength(railHandle, 6);
        }
      } else if (pktBuf[1] == 0x3D) {
        // Format B
        // L field in format B does not include L byte
        // Add 2 bytes because of extra sync vs mode T
        uint16_t len = packet_len_frame_b(pktBuf[2]) + 2;
        if (len > 12) {
          // Set packet length for mode C frame B
          RAIL_SetFixedLength(railHandle, len);
        } else {
          // Set packet length short - invalid header
          RAIL_SetFixedLength(railHandle, 6);
        }
      } else {
        // Unknown format, set short frame length to filter out packet
        RAIL_SetFixedLength(railHandle, 6);
      }
    } else {
      // Try 3of6 decoding and see if we get a decent packet length
      // mode T is always frame type A
      uint8_t nibble1 = decode3of6((pktBuf[0] & 0xFC) >> 2);
      uint8_t nibble2 = decode3of6(((pktBuf[0] & 0x03) << 4)  | ((pktBuf[1] & 0xF0) >> 4));

      if(nibble1 > 15 || nibble2 > 15) {
        // Unknown format, set short frame length to filter out packet
        RAIL_SetFixedLength(railHandle, 6);
      } else {
        // Set packet length
        uint16_t plen = packet_len_frame_a((nibble1 << 4) | nibble2);
        if (plen >= 12) {
          // plen is now in bytes, need to convert to 3of6 length, round up
          plen = ((plen * 3) + 1) / 2;
          RAIL_SetFixedLength(railHandle, plen);
        } else {
          // Set packet length short - invalid header
          RAIL_SetFixedLength(railHandle, 6);
        }
      }
    }
  }
}
