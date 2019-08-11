/*
File:   ManuvrConf.h
Author: J. Ian Lindsay


This is one of the files that the application author is required to provide.
This is where definition of (application or device)-specific parameters ought to go.
*/

#ifndef __MANUVR_FIRMWARE_DEFS_H
#define __MANUVR_FIRMWARE_DEFS_H

/*
* Particulars of this Manuvrable.
*/

#define EVENT_MANAGER_PREALLOC_COUNT     8
#define SCHEDULER_MAX_SKIP_BEFORE_RESET  6
#define PLATFORM_RNG_CARRY_CAPACITY     32
#define WITH_MBEDTLS
//#define WITH_BLIND_CRYPTO
//#define MANUVR_OVER_THE_WIRE
//#define MANUVR_SUPPORT_MQTT
//#define MANUVR_SUPPORT_COAP
#define MANUVR_CONSOLE_SUPPORT
#define MANUVR_STDIO
//#define MANUVR_SUPPORT_SERIAL
#define MANUVR_SUPPORT_TCPSOCKET
//#define MANUVR_SUPPORT_UDP
#define MANUVR_SUPPORT_I2C
  #define I2CADAPTER_MAX_QUEUE_DEPTH 16
  #define I2CADAPTER_PREALLOC_COUNT  4
#define CONFIG_MANUVR_SUPPORT_SPI
  #define CONFIG_SPIADAPTER_MAX_QUEUE_DEPTH    8
  #define CONFIG_SPIADAPTER_PREALLOC_COUNT     8
#define MANUVR_STORAGE
#define MANUVR_CBOR
//#define MANUVR_JSON
#define CONFIG_MANUVR_SENSOR_MGR
#define CONFIG_MANUVR_GPS_PIPE

#define CONFIG_MANUVR_BENCHMARKS
#define MANUVR_DEBUG
#define MANUVR_EVENT_PROFILER

// This is the string that identifies this Manuvrable to other Manuvrables. In MHB's case, this
//   will select the mEngine.
#define FIRMWARE_NAME     "radio-relay"

// This would be the version of the Manuvrable's firmware (this program).
#define VERSION_STRING    "0.1.1"

// Hardware is versioned. Manuvrables that are strictly-software should say -1 here.
#define HW_VERSION_STRING "2"


#endif  // __MANUVR_FIRMWARE_DEFS_H
