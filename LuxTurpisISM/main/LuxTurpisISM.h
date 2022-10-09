/* Compiler */
#include <inttypes.h>
#include <stdint.h>
#include <math.h>

/* ManuvrPlatform */
#include <ESP32.h>

/* CppPotpourri */
#include <CppPotpourri.h>
#include <AbstractPlatform.h>
#include <StringBuilder.h>
#include <ParsingConsole.h>
#include <StopWatch.h>
#include <uuid.h>
#include <Identity/Identity.h>
#include <Identity/IdentityUUID.h>
#include <SensorFilter.h>
#include <UARTAdapter.h>
#include <M2MLink/M2MLink.h>
#include <cbor-cpp/cbor.h>

/* ManuvrDrivers */
#include <ManuvrDrivers.h>

#ifndef __CHATTERBOX_H__
#define __CHATTERBOX_H__

// TODO: I _HaTe* that I have replicated this awful pattern of hard-coded
//   program versions (which are never updated) into so many projects. Finally
//   decide on a means of doing this that more-closely resembles the awesome
//   arrangement that I have at LTi for automatically binding the firmware
//   version to source-control.
#define TEST_PROG_VERSION           "1.0"


/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
/* Platform pins */
#define UART2_RX_PIN       16   // INPUT_PULLUP
#define UART2_TX_PIN       17   // OUTPUT
#define LED_R_PIN          25   // OUTPUT Active low
#define LED_G_PIN          26   // OUTPUT Active low


/*******************************************************************************
* Invariant software parameters
*******************************************************************************/


/*******************************************************************************
* Types
*******************************************************************************/


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/
extern MOVI movi;   // The voice box.


/*******************************************************************************
* Externs to software singletons
*******************************************************************************/
// TODO: If you replicate this any further, you deserve the extra work you make
//   for yourself. This is a terrible pattern for so many reasons...
extern M2MLink* mlink_local;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
int8_t report_fault_condition(int8_t);


#endif    // __HEAT_PUMP_H__
