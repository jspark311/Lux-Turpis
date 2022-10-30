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
#include <GPSWrapper.h>

/* ManuvrDrivers */
#include <ManuvrDrivers.h>

#ifndef __LUX_TURPIS_H__
#define __LUX_TURPIS_H__

// TODO: I _HaTe* that I have replicated this awful pattern of hard-coded
//   program versions (which are never updated) into so many projects. Finally
//   decide on a means of doing this that more-closely resembles the awesome
//   arrangement that I have at LTi for automatically binding the firmware
//   version to source-control.
#define TEST_PROG_VERSION           "1.0"


/*******************************************************************************
* Pin definitions and hardware constants.    ESP32 Pin quirks:
*                                            ----------------------------
*                                             5  Outputs PWM at boot
*                                            12  Boot will fail if pulled high
*                                            14  Outputs PWM at boot
*                                            15  Outputs PWM at boot
*                                            34  INPUT only, no built-in pullup
*                                            35  INPUT only, no built-in pullup
*                                            36  INPUT only, no built-in pullup
*                                            39  INPUT only, no built-in pullup
*******************************************************************************/
/* Platform pins */
/* LORA0 is the 915MHz module */
/* LORA1 is the 868MHz module */
#define SPI_MOSI_PIN         5  // OUTPUT
#define UNDEFD_PIN_1        12  // (Boot will fail if pulled high)
#define UNDEFD_PIN_2        14  // (Outputs PWM at boot)
#define UNDEFD_PIN_3        15  // (Outputs PWM at boot)
#define SPI_CLK_PIN         16  // OUTPUT
#define LORA1_CS_PIN        17  // OUTPUT
#define SPI_MISO_PIN        18  // INPUT
#define LORA1_RESET_PIN     19  // OUTPUT
#define LORA1_DIO0_PIN      21  // INPUT
#define LORA1_DIO1_PIN      22  // INPUT
#define LORA1_DIO2_PIN      23  // INPUT
#define SCL_PIN             26  // I2C0
#define SDA_PIN             27  // I2C0
#define LORA0_RESET_PIN     32  // OUTPUT
#define LORA0_CS_PIN        33  // OUTPUT
#define LORA0_DIO1_PIN      34  // INPUT
#define LORA0_DIO2_PIN      35  // INPUT
#define UART2_RX_PIN        36  // INPUT GPS TX pin
#define LORA0_DIO0_PIN      39  // INPUT


/*******************************************************************************
* Invariant software parameters
*******************************************************************************/


/*******************************************************************************
* Types
*******************************************************************************/

/*
* Options object
*/
class LuxTurpisOpts {
  public:
    LuxTurpisOpts() {};
    LuxTurpisOpts(const LuxTurpisOpts* o) {};

  private:
};



class LuxTurpis {
  public:
    LuxTurpis(const LuxTurpisOpts* o) : _opts(o) {}
    ~LuxTurpis() {};

    void printDebug(StringBuilder*);


  private:
    LuxTurpisOpts _opts;
};



/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/
extern SPIAdapter spi_bus;
extern I2CAdapter i2c0;
extern GPSWrapper gps;


/*******************************************************************************
* Externs to hardware resources
*******************************************************************************/


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


#endif    // __LUX_TURPIS_H__
