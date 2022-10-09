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
* Pin definitions and hardware constants.
*******************************************************************************/
/* Platform pins */

#define UNDEFD_PIN_0         5  // (Outputs PWM at boot)
#define UNDEFD_PIN_1        12  // (Boot will fail if pulled high)
#define UNDEFD_PIN_2        14  // (Outputs PWM at boot)
#define UNDEFD_PIN_3        15  // (Outputs PWM at boot)

#define UART2_RX_PIN        16  // INPUT     GPS TX pin
#define SPI_MISO_PIN        17  // INPUT
#define SPI_MOSI_PIN        18  // OUTPUT
#define SPI_CLK_PIN         19  // OUTPUT
#define SDA_PIN             21  // I2C0
#define SCL_PIN             22  // I2C0
#define LORA0_CS_PIN        23  // OUTPUT
#define LORA0_RESET_PIN     25  // OUTPUT
#define LORA0_DIO0_PIN      26  // INPUT
#define LORA0_DIO1_PIN      27  // INPUT
#define LORA0_DIO2_PIN      34  // INPUT (no built-in pullup)
#define LORA1_CS_PIN        32  // OUTPUT
#define LORA1_RESET_PIN     33  // OUTPUT
#define LORA1_DIO0_PIN      35  // INPUT (no built-in pullup)
#define LORA1_DIO1_PIN      36  // INPUT (no built-in pullup)
#define LORA1_DIO2_PIN      39  // INPUT (no built-in pullup)


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


#endif    // __LUX_TURPIS_H__
