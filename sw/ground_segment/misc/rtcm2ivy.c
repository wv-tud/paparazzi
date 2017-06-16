/*
 * Copyright (C) 2016 Wilco Vlenterie, Anand Sundaresan.
 * Contact: Anand Sundaresan <nomail@donotmailme.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * \file rtcm2ivy.c
 * \brief RTCM3 GPS packets to Ivy for DGPS and RTK
 *
 * This communicates with an RTCM3 GPS receiver like an
 * ublox M8P. This then forwards the Observed messages
 * over the Ivy bus to inject them for DGPS and RTK positioning.
 */

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <rtcm3.h>              // Used to decode RTCM3 messages
#include <CRC24Q.h>             // Used to verify CRC checks
#include <math/pprz_geodetic_float.h>

#include "std.h"
#include "serial_port.h"

void UbxSend_CFG_TMODE3(uint8_t ubx_version, uint8_t ubx_res1, uint16_t ubx_flags, int32_t ubx_ececfxorlat, int32_t ubx_ececfyorlon, int32_t ubx_ececfzoralt, int8_t ubx_ececfxorlathp, int8_t ubx_ececfyorlonhp, int8_t ubx_ececfzoralthp, uint8_t ubx_res2, uint32_t ubx_fixedposacc, uint32_t ubx_svinmindur, uint32_t ubx_svinacclimit, uint32_t ubx_res3, uint32_t ubx_res4);

/** Used variables **/
struct SerialPort *serial_port;

/* ubx structure definitions*/
msg_state_t msg_state;
rtcm3_msg_callbacks_node_t rtcm3_1005_node;
rtcm3_msg_callbacks_node_t rtcm3_1077_node;
rtcm3_msg_callbacks_node_t rtcm3_1087_node;

rtcm3_msg_callbacks_node_t ubx_nav_svin_node;

/** Default values **/
uint8_t ac_id         = 0;
uint32_t msg_cnt      = 0;
char *serial_device   = "/dev/ttyACM0";
uint32_t serial_baud  = B9600;
uint32_t packet_size  = 100;    // 802.15.4 (Series 1) XBee 100 Bytes payload size
uint32_t ivy_size     = 0;
uint16_t min_dur_s    = 240;
uint16_t min_acc_m   = 1.5;
bool svin = false;

/* For sending ubx msg to ground station gps */
uint8_t send_ck_a, send_ck_b;

#define PACKET_MAX_SIZE	512
#define IVY_MSG_HEAD    "rtcm2ivy RTCM_INJECT"

/** Debugging options */
bool verbose          = FALSE;
bool logger           = FALSE;

#define printf_debug    if(verbose == TRUE) printf

FILE  *pFile;

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255"; // 192.168.1.255   127.255.255.255
#endif

/*
 * Read bytes from the uBlox UART connection
 * This is a wrapper functions used in the librtcm3 library
 */
static uint32_t uart_read(unsigned char(*buff)[], uint32_t n)  //, void *context __attribute__((unused))
{
  int ret = read(serial_port->fd, buff, n);
  if (ret > 0) {
    return ret;
  } else {
    return 0;
  }
}

static void ivy_send_message(uint8_t packet_id, uint8_t len, uint8_t msg[])
{
  char number[5];
  char gps_packet[PACKET_MAX_SIZE];
  uint8_t cpt;
  uint8_t offset=0;

  while (offset < len) { // fragment if necessary

    snprintf(gps_packet, ivy_size, IVY_MSG_HEAD" %d %d", packet_id, msg[offset]);

    cpt = 1;
    while ((cpt < (packet_size - 5)) && (cpt < (len-offset))) {
      snprintf(number, 5, ",%d", msg[cpt+offset]); // coma + (000..255) + '\0' = 5 chars
      strcat(gps_packet, number);
      cpt++;
    }

    printf_debug("%s\n\n", gps_packet);
    IvySendMsg("%s", gps_packet);
    offset += (packet_size-5);

    if (logger == TRUE) {
      pFile = fopen("./RTCM3_log.txt", "a");
      fprintf(pFile, "%s\n", gps_packet);
      fclose(pFile);
    }
    printf_debug("Ivy send: %s\n", gps_packet);
  }
}

/*
 * Callback for the 1005 message to send it trough RTCM_INJECT
 */
struct EcefCoor_f posEcef;
struct LlaCoor_f  posLla;

static void rtcm3_1005_callback(uint8_t len, uint8_t msg[])
{
  if (len > 0) {
    if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
      ivy_send_message(RTCM3_MSG_1005, len, msg);
      msg_cnt++;
      u16 StaId      = RTCMgetbitu(msg, 24 + 12, 12);
      u8 ItRef       = RTCMgetbitu(msg, 24 + 24, 6);
      u8 indGPS      = RTCMgetbitu(msg, 24 + 30, 1);
      u8 indGlonass  = RTCMgetbitu(msg, 24 + 31, 1);
      u8 indGalileo  = RTCMgetbitu(msg, 24 + 32, 1);
      u8 indRefS     = RTCMgetbitu(msg, 24 + 33, 1);
      posEcef.x      = RTCMgetbits_38(msg, 24 + 34) * 0.0001;
      posEcef.y      = RTCMgetbits_38(msg, 24 + 74) * 0.0001;
      posEcef.z      = RTCMgetbits_38(msg, 24 + 114) * 0.0001;
      lla_of_ecef_f(&posLla, &posEcef);
      printf_debug("Lat: %f, Lon: %f, Alt: %f\n", posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
      // Send spoof gpsd message to GCS to plot groundstation position
      IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f", "ground", "FLIGHT_PARAM", "GCS", 0.0, 0.0, 0.0,
                 posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, 0.0, 0.0, posLla.alt, 0.0, 0.0, 0.0, 0,  0.0);
      // Send UBX_RTK_GROUNDSTATION message to GCS for RTK info
      IvySendMsg("%s %s %s %i %i %i %i %i %i %f %f %f", "ground", "UBX_RTK_GROUNDSTATION", "GCS", StaId, ItRef, indGPS,
                 indGlonass, indGalileo, indRefS, posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
    } else {
      printf("Skipping 1005 message (CRC check failed)\n");
    }
  }
  printf_debug("Parsed 1005 callback\n");
}

/*
 * Callback for the 1077 message to send it trough RTCM_INJECT
 */
static void rtcm3_1077_callback(uint8_t len, uint8_t msg[])
{
  if (len > 0) {
    if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
      ivy_send_message(RTCM3_MSG_1077, len, msg);
      msg_cnt++;
    } else {
      ivy_send_message(RTCM3_MSG_1077, len, msg);
      printf("Skipping 1077 message (CRC check failed)\n");
    }
  }
  printf_debug("Parsed 1077 callback\n");
}

/*
 * Callback for the 1087 message to send it trough RTCM_INJECT
 */
static void rtcm3_1087_callback(uint8_t len, uint8_t msg[])
{
  if (len > 0) {
    if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
      ivy_send_message(RTCM3_MSG_1087, len, msg);
      msg_cnt++;
    } else {
      printf("Skipping 1087 message (CRC check failed)\n");
    }
  }
  printf_debug("Parsed 1087 callback\n");
}


/*
 * Callback for UBX survey-in message
 */
static void ubx_navsvin_callback(uint8_t len, uint8_t msg[])
{
  if (len > 0) {
    u32 iTow      = UBX_NAV_SVIN_ITOW(msg);
    u32 dur       = UBX_NAV_SVIN_dur(msg);
    float meanAcc = (float) 0.1 * UBX_NAV_SVIN_meanACC(msg);
    u8 valid      = UBX_NAV_SVIN_Valid(msg);
    u8 active     = UBX_NAV_SVIN_Active(msg);
    printf("\033[A\33[2K\r");
    printf("iTow: %10u \t dur: %5u \t meaAcc: %15.5f \t valid: %1u \t active: %1u\n", iTow, dur, meanAcc, valid, active);
  }
}
/**
 * Parse the tty data when bytes are available
 */
static gboolean parse_device_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
  unsigned char buff[1000];
  int c;
  c = uart_read(&buff, 1);
  if (c > 0) {                // Have we read anything?
    if (msg_state.msg_class == RTCM_CLASS) {  // Are we already reading a RTCM message?
      rtcm3_process(&msg_state, buff[0]);     // If so continue reading RTCM
    } else if (msg_state.msg_class == UBX_CLASS) { // Are we already reading a UBX message?
      ubx_process(&msg_state, buff[0]);       // If so continue reading UBX
    } else {
      msg_state.state = UNINIT;           // Not reading anything yet
      rtcm3_process(&msg_state, buff[0]);     // Try to process preamble as RTCM
      if (msg_state.msg_class != RTCM_CLASS) { // If it wasn't a RTCM preamble
        ubx_process(&msg_state, buff[0]);     // Check for UBX preamble
      }
    }
  }
  return TRUE;
}

/** Print the program help */
void print_usage(int argc __attribute__((unused)), char **argv)
{
  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h, --help                Display this help\n"
    "   -v, --verbose             Verbosity enabled\n"
    "   -l, --logger              Save RTCM3 messages to log\n\n"

    "   -s, --survey-in           Enable survey-in mode, uses <time> and <accuracy>\n"
    "   -t <time>                 Minimum survey-in time in seconds\n"
    "   -a <accuracy>             Minimum survey-in accuracy in milimeters\n"
    "   -d <device>               The GPS device(default: /dev/ttyACM0)\n"
    "   -b <baud_rate>            The device baud rate(default: B9600)\n"
    "   -p <packet_size>          The payload size (default:100, max:4146))\n\n";
  fprintf(stderr, usage, argv[0]);
}

int main(int argc, char **argv)
{
  ivy_size = packet_size + strlen(IVY_MSG_HEAD) + 5;  // Header+blank+(000..255)packetId+blank

  // Parse the options from cmdline
  char c;
  while ((c = getopt(argc, argv, "hvlst:a:p:d:b:i:")) != EOF) {
    switch (c) {
      case 'h':
        print_usage(argc, argv);
        exit(EXIT_SUCCESS);
        break;
      case 'v':
        verbose = TRUE;
        break;
      case 'l':
        logger = TRUE;
        break;
      case 's':
          svin = TRUE;
          break;
      case 't':
          min_dur_s = atoi(optarg);
          break;
      case 'a':
          min_acc_m = atoi(optarg);
          break;
      case 'p':
        packet_size = atoi(optarg);
        ivy_size = packet_size + strlen(IVY_MSG_HEAD) + 5;  // Header+blank+(000..255)packetId+blank
        if (ivy_size > PACKET_MAX_SIZE) {
          printf("%d exceed max size %d\n", ivy_size, PACKET_MAX_SIZE);
          exit(EXIT_FAILURE);
        }
        break;
      case 'd':
          serial_device = optarg;
          break;
      case 'b':
        serial_baud = atoi(optarg);
        break;
      case 'i':
        ac_id = atoi(optarg);
        break;
      case '?':
        if (optopt == 'p' || optopt == 'd' || optopt == 'b' || optopt == 'i' || optopt == 't' || optopt == 'a') {
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        } else if (isprint(optopt)) {
          fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        } else {
          fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
        print_usage(argc, argv);
        exit(EXIT_FAILURE);
      default:
        abort();
    }
  }
  // Create the Ivy Client
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit("Paparazzi server", "Paparazzi server READY", 0, 0, 0, 0);
  IvyStart(ivy_bus);

  // Start the tty device
  printf_debug("Opening tty device %s...\n", serial_device);
  serial_port = serial_port_new();
  int ret = serial_port_open_raw(serial_port, serial_device, serial_baud);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    serial_port_free(serial_port);
    exit(EXIT_FAILURE);
  }

  // Setup RTCM3 callbacks
  printf_debug("Setup RTCM3 callbacks...\n");
  msg_state_init(&msg_state);
  rtcm3_register_callback(&msg_state, RTCM3_MSG_1005, &rtcm3_1005_callback, &rtcm3_1005_node);
  rtcm3_register_callback(&msg_state, RTCM3_MSG_1077, &rtcm3_1077_callback, &rtcm3_1077_node);
  rtcm3_register_callback(&msg_state, RTCM3_MSG_1087, &rtcm3_1087_callback, &rtcm3_1087_node);

  rtcm3_register_callback(&msg_state, UBX_NAV_SVIN, &ubx_navsvin_callback, &ubx_nav_svin_node);


  // Add IO watch for tty connection
  printf_debug("Adding IO watch...\n");
  GIOChannel *sk = g_io_channel_unix_new(serial_port->fd);
  g_io_add_watch(sk, G_IO_IN, parse_device_data, NULL);

  // Configuring the chip for TMODE3 (svin parameters)
  if(svin){
      //  uint16_t ubx_flags;
      uint8_t lla = 1;                              ///< 0: ECEF,  1: LLA
      uint8_t mode = 1;                             ///< 0: Disabled,  1: Suvery-in  2: Fixed mode
      uint16_t ubx_flags = (lla<<8)+mode;           // FIXME : mind the reserve bytes get shifted
      uint32_t svinacclimit = min_acc_m * 10000;    ///< Minimum accuracy in 0.1 mm
      uint32_t svinmindur = min_dur_s;              ///< Minimum duration in seconds
      UbxSend_CFG_TMODE3(0,0, ubx_flags,0,0,0,0,0,0,0,0,svinmindur,svinacclimit,0,0);
  }

  // Run the main loop
  printf_debug("Started rtcm2ivy for aircraft id %d!\n", ac_id);
  g_main_loop_run(ml);

  return 0;
}

uint8_t uart_write(uint8_t *p);
void ubx_send_bytes(uint8_t len, uint8_t *bytes);
void ubx_send_1byte(uint8_t byte);
void ubx_trailer(void);
void ubx_header(uint8_t nav_id, uint8_t msg_id, uint16_t len);

#define UBX_CFG_MSG_ID 0x01
#define UBX_CFG_ID 0x06
#define UBX_CFG_TMODE3_ID 0x71

static inline void UbxSend_CFG_MSG(uint8_t ubx_class, uint8_t ubx_msgid, uint8_t ubx_rate) {
  ubx_header(UBX_CFG_ID, UBX_CFG_MSG_ID, 3);
  uint8_t _class = ubx_class;
  ubx_send_bytes(1, (uint8_t*)&_class);
  uint8_t _msgid = ubx_msgid;
  ubx_send_bytes( 1, (uint8_t*)&_msgid);
  uint8_t _rate = ubx_rate;
  ubx_send_bytes( 1, (uint8_t*)&_rate);
  ubx_trailer();
}
// Configuring the ground station survery in parameters.
void UbxSend_CFG_TMODE3(uint8_t ubx_version, uint8_t ubx_res1, uint16_t ubx_flags, int32_t ubx_ececfxorlat, int32_t ubx_ececfyorlon, int32_t ubx_ececfzoralt, int8_t ubx_ececfxorlathp, int8_t ubx_ececfyorlonhp, int8_t ubx_ececfzoralthp, uint8_t ubx_res2, uint32_t ubx_fixedposacc, uint32_t ubx_svinmindur, uint32_t ubx_svinacclimit, uint32_t ubx_res3, uint32_t ubx_res4) {

   ubx_header(UBX_CFG_ID, UBX_CFG_TMODE3_ID, 40);
   uint8_t _version = ubx_version;              ubx_send_bytes(1, (uint8_t*)&_version);
   uint8_t _res1 = ubx_res1;                    ubx_send_bytes(1, (uint8_t*)&_res1);
   uint16_t _flags = ubx_flags;                 ubx_send_bytes(2, (uint8_t*)&_flags);
   int32_t _ececfxorlat = ubx_ececfxorlat;      ubx_send_bytes(4, (uint8_t*)&_ececfxorlat);
   int32_t _ececfyorlon = ubx_ececfyorlon;      ubx_send_bytes(4, (uint8_t*)&_ececfyorlon);
   int32_t _ececfzoralt = ubx_ececfzoralt;      ubx_send_bytes(4, (uint8_t*)&_ececfzoralt);
   int8_t _ececfxorlathp = ubx_ececfxorlathp;   ubx_send_bytes(1, (uint8_t*)&_ececfxorlathp);
   int8_t _ececfyorlonhp = ubx_ececfyorlonhp;   ubx_send_bytes(1, (uint8_t*)&_ececfyorlonhp);
   int8_t _ececfzoralthp = ubx_ececfzoralthp;   ubx_send_bytes(1, (uint8_t*)&_ececfzoralthp);
   uint8_t _res2 = ubx_res2;                    ubx_send_bytes(1, (uint8_t*)&_res2);
   uint32_t _fixedposacc = ubx_fixedposacc;     ubx_send_bytes(4, (uint8_t*)&_fixedposacc);
   uint32_t _svinmindur = ubx_svinmindur;       ubx_send_bytes(4, (uint8_t*)&_svinmindur);
   uint32_t _svinacclimit = ubx_svinacclimit;   ubx_send_bytes(4, (uint8_t*)&_svinacclimit);
   uint32_t _res3 = ubx_res3;                   ubx_send_bytes(4, (uint8_t*)&_res3);
   uint32_t _res4 = ubx_res4;                   ubx_send_bytes(4, (uint8_t*)&_res4); // 8 seperate bytes
   ubx_trailer();
}

void ubx_header(uint8_t nav_id, uint8_t msg_id, uint16_t len) // writes the first six bytes of ubx msg package
{
    uint8_t sync1 = UBX_PREAMBLE1;
    uint8_t sync2 = UBX_PREAMBLE2;
    uart_write(&sync1);
    uart_write(&sync2);
    send_ck_a = 0;
    send_ck_b = 0;
    ubx_send_1byte(nav_id);
    ubx_send_1byte(msg_id);
    ubx_send_1byte((uint8_t)(len&0xFF));
    ubx_send_1byte((uint8_t)(len>>8));
}

void ubx_trailer(void)
{
    uart_write(&send_ck_a);
    uart_write(&send_ck_b);
}

void ubx_send_1byte(uint8_t byte)
{
    uart_write(&byte);
    send_ck_a = send_ck_a + byte;
    send_ck_b = send_ck_b + send_ck_a;
}

void ubx_send_bytes(uint8_t len, uint8_t *bytes)
{
    int i;
    for (i = 0; i < len; i++) {
        ubx_send_1byte(bytes[i]);
    }
}

uint8_t uart_write(uint8_t *p)
{
//  printf("Size of the buffer: %i, Buffer content:%0x \n",sizeof(*p),*p);
//    unsigned char buff1 = *p;
//    printf("Content in buffer: %0x \n", buff1);
    int ret = 0;
    do{
        ret = write( serial_port->fd, p, 1);
//      serial_port_flush_output(serial_port->fd);
      } while(ret < 1 && errno == EAGAIN); //FIXME: max retry

    if(ret > 0){
        //printf ("Byte content: 0x%X \n", *p);
        return ret;
    }
    else
        return 0;
}
