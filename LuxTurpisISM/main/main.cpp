/* Local includes */
#include "LuxTurpisISM.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"

#include "esp_http_client.h"
#include "mqtt_client.h"

#ifdef __cplusplus
}
#endif


#define DEFAULT_SCAN_LIST_SIZE  16


/*******************************************************************************
* TODO: Pending mitosis into a header file....
*******************************************************************************/
IdentityUUID ident_uuid("LuxTurpisISM", (char*) "765a2853-7cd1-4e12-9e84-f28ac1cee27f");


/*******************************************************************************
* Globals
*******************************************************************************/
const I2CAdapterOptions i2c0_opts(
  0,        // Device number
  SDA_PIN,  // (sda)
  SCL_PIN,  // (scl)
  0,        // No pullups. Internals are too weak. They are provided on board.
  100000
);

const SX1276Opts sx1276_opts_0(
  LORA0_RESET_PIN,  // Reset
  LORA0_CS_PIN,     // CS
  LORA0_DIO0_PIN,   // D0
  LORA0_DIO1_PIN,   // D1
  LORA0_DIO2_PIN,   // D2
  LORABand::BAND_915
);


const SX1276Opts sx1276_opts_1(
  LORA1_RESET_PIN,  // Reset
  LORA1_CS_PIN,     // CS
  LORA1_DIO0_PIN,   // D0
  LORA1_DIO1_PIN,   // D1
  LORA1_DIO2_PIN,   // D2
  LORABand::BAND_868
);

/*
* This UART is connected to the forked GPS TX pin. Another system has direct
*   control of the GPS (including power), but we can still take data on an
*   opportunistic basis.
*/
UARTOpts uart2_opts {
  .bitrate       = 9600,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};

static const char* TAG = "main-cpp";
static const char* console_prompt_str = "LuxTurpisISM # ";

ParsingConsole console(128);
ESP32StdIO console_uart;
UARTAdapter gps_uart(2, UART2_RX_PIN, 255, 255, 255, 128, 128);
SPIAdapter spi_bus(1, SPI_CLK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, 8);
I2CAdapter i2c0(&i2c0_opts);
GPSWrapper gps;

//TMP102 temperature_sensor;    // Optional temperature monitoring.

/* Profiling data */
StopWatch stopwatch_main_loop_time;

/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().


/*******************************************************************************
* Process functions called from the main service loop.
*******************************************************************************/


/*******************************************************************************
* Link callbacks
*******************************************************************************/


/*******************************************************************************
* Console callbacks
*******************************************************************************/

/* Direct console shunt */
int callback_help(StringBuilder* txt_ret, StringBuilder* args) {
  return console.console_handler_help(txt_ret, args);
}

/* Direct console shunt */
int callback_console_tools(StringBuilder* txt_ret, StringBuilder* args) {
  return console.console_handler_conf(txt_ret, args);
}

/* Direct console shunt */
int console_callback_movi(StringBuilder* txt_ret, StringBuilder* args) {
  return movi.console_handler(txt_ret, args);
}

/**
* @page console-handlers
* @section uart-tools UART tools
*
* This is the console handler for debugging the operation of the UART hardware.
*
* @subsection arguments Arguments
* Argument | Purpose | Required
* -------- | ------- | --------
* 1        | UartID  | No (lists UARTs if not provided)
* 2        | Action  | No (prints debugging information for specified UART if not provided)
* 3        | Action-Specific | No
*
* @subsection cmd-actions Actions
* Action   | Description | Additional arguments
* -------- | ----------- | --------------------
* `init`   | Enable the UART, claim the pins, initialize associated memory, and begin operation. | None
* `deinit` | Disable the UART, release the pins, and wipe associated memory. | None
* `poll`   | Manually invoke the UART driver's `poll()` function. | None
* `read`   | Reads all available data from the UART and renders it to the console. | None
*/
int callback_uart_tools(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = 0;
  bool print_uarts = true;
  if (0 < args->count()) {
    int uart_num = args->position_as_int(0);
    //UARTAdapter* uart = &console_uart;
    //UARTOpts* default_opts = (UARTOpts*) &console_uart_opts;
    //switch (uart_num) {
    //  case 0:
    //  case 2:
    //    print_uarts = false;
    //    if (1 < args->count()) {
    //      char* cmd = args->position_trimmed(1);
    //      if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    //        //UARTOpts* opts = uart->uartOpts();
    //        text_return->concatf("UART%u.init() returns %d.\n", uart_num, uart->init(default_opts));
    //      }
    //      else if (0 == StringBuilder::strcasecmp(cmd, "deinit")) {
    //        text_return->concatf("UART%u.deinit() returns %d.\n", uart_num, uart->deinit());
    //      }
    //      //else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    //      //  text_return->concatf("UART%u.reset() returns %d.\n", uart_num, uart->reset());
    //      //}
    //      //else if (0 == StringBuilder::strcasecmp(cmd, "bitrate")) {
    //      //}
    //      else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
    //        text_return->concatf("UART%u.poll() returns %d.\n", uart_num, uart->poll());
    //      }
    //      else if (0 == StringBuilder::strcasecmp(cmd, "read")) {
    //        StringBuilder rx;
    //        uart->read(&rx);
    //        text_return->concatf("UART%u.read() returns %u bytes:\n", uart_num, rx.length());
    //        rx.printDebug(text_return);
    //      }
    //      else {
    //        ret = -1;
    //      }
    //    }
    //    else {
    //      uart->printDebug(text_return);
    //    }
    //    break;
    //  default:
    //    text_return->concat("Unknown UART.\n");
    //    break;
    //}
  }
  if (print_uarts) {
    text_return->concat("Supported UARTs:\n\t0: CONSOLE\n\t2: MOVI UART\n");
  }
  return ret;
}



#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Wifi utilities
*******************************************************************************/

void printPeerDescript(StringBuilder* output, wifi_ap_record_t* ap_info) {
  output->concatf("\n\t[%s]\tRSSI: %d\tCHAN: %d\n", ap_info->ssid, ap_info->rssi, ap_info->primary);
  output->concat("\t\tAuthmode: ");
  switch (ap_info->authmode) {
    case WIFI_AUTH_OPEN:             output->concat("OPEN");             break;
    case WIFI_AUTH_WEP:              output->concat("WEP");              break;
    case WIFI_AUTH_WPA_PSK:          output->concat("WPA_PSK");          break;
    case WIFI_AUTH_WPA2_PSK:         output->concat("WPA2_PSK");         break;
    case WIFI_AUTH_WPA_WPA2_PSK:     output->concat("WPA_WPA2_PSK");     break;
    case WIFI_AUTH_WPA2_ENTERPRISE:  output->concat("WPA2_ENTERPRISE");  break;
    case WIFI_AUTH_WPA3_PSK:         output->concat("WPA3_PSK");         break;
    //case WIFI_AUTH_WPA2_WPA3_PSK:    output->concat("WPA2_WPA3_PSK");    break;
    default:    output->concat("WIFI_AUTH_UNKNOWN");    break;
  }
  if (ap_info->authmode != WIFI_AUTH_WEP) {
    output->concat("\n\t\tPairwise: ");
    switch (ap_info->pairwise_cipher) {
      case WIFI_CIPHER_TYPE_NONE:       output->concat("NONE");      break;
      case WIFI_CIPHER_TYPE_WEP40:      output->concat("WEP40");     break;
      case WIFI_CIPHER_TYPE_WEP104:     output->concat("WEP104");    break;
      case WIFI_CIPHER_TYPE_TKIP:       output->concat("TKIP");      break;
      case WIFI_CIPHER_TYPE_CCMP:       output->concat("CCMP");      break;
      case WIFI_CIPHER_TYPE_TKIP_CCMP:  output->concat("TKIP_CCMP"); break;
      default:                          output->concat("UNKNOWN");   break;
    }
    output->concat("\tGroup: ");
    switch (ap_info->group_cipher) {
      case WIFI_CIPHER_TYPE_NONE:       output->concat("NONE");      break;
      case WIFI_CIPHER_TYPE_WEP40:      output->concat("WEP40");     break;
      case WIFI_CIPHER_TYPE_WEP104:     output->concat("WEP104");    break;
      case WIFI_CIPHER_TYPE_TKIP:       output->concat("TKIP");      break;
      case WIFI_CIPHER_TYPE_CCMP:       output->concat("CCMP");      break;
      case WIFI_CIPHER_TYPE_TKIP_CCMP:  output->concat("TKIP_CCMP"); break;
      default:                          output->concat("UNKNOWN");   break;
    }
  }
  output->concat("\n");
}


/* Initialize Wi-Fi as sta and set scan method */
static void wifi_scan() {
  uint16_t number = DEFAULT_SCAN_LIST_SIZE;
  wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
  uint16_t ap_count = 0;
  memset(ap_info, 0, sizeof(ap_info));

  esp_wifi_scan_start(NULL, true);
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
  c3p_log(LOG_LEV_NOTICE, TAG, "Total APs scanned = %u", ap_count);

  for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
    StringBuilder peer_desc;
    printPeerDescript(&peer_desc, &ap_info[i]);
    c3p_log(LOG_LEV_INFO, TAG, &peer_desc);
  }
}


/*******************************************************************************
* Support functions
*******************************************************************************/


/*******************************************************************************
* Threads
*******************************************************************************/

/**
* This is the Manuvr thread that runs constatntly, and would be the main loop
*   of a single-threaded program.
*/
void manuvr_task(void* pvParameter) {
  while (1) {
    uint32_t millis_now = millis();
    bool should_sleep = true;
    while (0 < spi_bus.service_callback_queue()) {
      should_sleep = false;
    }
    should_sleep = (0 < console_uart.poll());
    should_sleep = (0 < gps_uart.poll());

    if (should_sleep) {}
    platform.yieldThread();
  }
}



/*******************************************************************************
* Setup function. This will be the entry-point for our code from ESP-IDF's
*   boilerplate. Since we don't trust the sdkconfig to have specified a stack
*   of appropriate depth, we do our setup, launch any threads we want, and the
*   let this thread die.
*******************************************************************************/
void app_main() {
  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform_init();
  boot_time = millis();

  StringBuilder ptc("LuxTurpisISM ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");

  /* Start the console UART and attach it to the console. */
  console_uart.readCallback(&console);    // Attach the UART to console...
  console.setOutputTarget(&console_uart); // ...and console to UART.
  console.setTXTerminator(LineTerm::CRLF); // Best setting for "idf.py monitor"
  console.setRXTerminator(LineTerm::LF);   // Best setting for "idf.py monitor"
  console.setPromptString(console_prompt_str);
  console.emitPrompt(true);
  console.localEcho(true);
  console.printHelpOnFail(true);
  console.init();

  console.defineCommand("help",        '?',  "Prints help to console.", "[<specific command>]", 0, callback_help);
  console.defineCommand("console",     '\0', "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  platform.configureConsole(&console);
  console.defineCommand("str",         '\0', "Storage tools", "", 0, console_callback_esp_storage);
  console.defineCommand("uart",        'u',  "UART tools", "<adapter> [init|deinit|reset|poll]", 0, callback_uart_tools);

  spi_bus.init();
  i2c0.init();
  ptc.concatf("gps.init()        \t  %d\n", gps.init());
  gps_uart.readCallback(&gps);
  ptc.concatf("gps_uart.init()   \t %d\n", gps_uart.init(&uart2_opts));

  // Write our boot log to the UART.
  console.printToLog(&ptc);

  // Spawn worker thread for Manuvr's use.
  xTaskCreate(manuvr_task, "_manuvr", 32768, NULL, (tskIDLE_PRIORITY), NULL);

  // TODO: Spawn worker thread for wifi use.
  // Setup Wifi peripheral in station mode.
  //ESP_ERROR_CHECK(esp_netif_init());
  //ESP_ERROR_CHECK(esp_event_loop_create_default());
  //esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
  //assert(sta_netif);
  //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  //ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  //ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  //ESP_ERROR_CHECK(esp_wifi_start());
  //wifi_scan();

  // Note the time it took to do setup, and allow THIS thread to gracefully terminate.
  config_time = millis();
}

#ifdef __cplusplus
}
#endif
