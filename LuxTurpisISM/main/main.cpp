#include <math.h>

/* Local includes */
#include "Chatterbox.h"

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
IdentityUUID ident_uuid("Chatterbox", (char*) "2f0891e1-ba39-4779-9e7d-17c771ced179");



/*******************************************************************************
* Globals
*******************************************************************************/
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

M2MLinkOpts link_opts(
  100,   // ACK timeout is 100ms.
  2000,  // Send a KA every 2s.
  2048,  // MTU for this link is 2 kibi.
  TCode::CBOR,   // Payloads should be CBOR encoded.
  // This side of the link will send a KA while IDLE, and
  //   allows remote log write.
  (M2MLINK_FLAG_SEND_KA | M2MLINK_FLAG_ALLOW_LOG_WRITE)
);


static const char* TAG         = "main-cpp";
const char* console_prompt_str = "Chatterbox # ";
ParsingConsole console(128);
ESP32StdIO console_uart;

UARTAdapter movi_uart(2, UART2_RX_PIN, UART2_TX_PIN, 255, 255, 256, 256);
MOVI movi(&movi_uart);

M2MLink* mlink_local = nullptr;

/* Profiling data */
StopWatch stopwatch_main_loop_time;

/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.
uint32_t ping_req_time     = 0;
uint32_t ping_nonce        = 0;


/*******************************************************************************
* Process functions called from the main service loop.
*******************************************************************************/

int8_t report_fault_condition(int8_t fault) {
  int8_t ret = 0;
  c3p_log(LOG_LEV_CRIT, TAG, "Chatterbox fault: %d\n", ret);
  return ret;
}



/*******************************************************************************
* Link callbacks
*******************************************************************************/
void link_callback_state(M2MLink* cb_link) {
  StringBuilder log;
  c3p_log(LOG_LEV_NOTICE, TAG, "Link (0x%x) entered state %s\n", cb_link->linkTag(), M2MLink::sessionStateStr(cb_link->getState()));
}


void link_callback_message(uint32_t tag, M2MMsg* msg) {
  StringBuilder log;
  KeyValuePair* kvps_rxd = nullptr;
  bool dump_msg_debug = true;
  log.concatf("link_callback_message(Tag = 0x%08x, ID = %u):\n", tag, msg->uniqueId());
  msg->getPayload(&kvps_rxd);
  if (kvps_rxd) {
    char* fxn_name = nullptr;
    if (0 == kvps_rxd->valueWithKey("fxn", &fxn_name)) {
      if (0 == strcmp("PING", fxn_name)) {
        //dump_msg_debug = false;
        // Counterparty may have replied to our ping. If not, reply logic will
        //   handle the response.
        if (ping_nonce) {
          if (ping_nonce == msg->uniqueId()) {
            log.concatf("\tPing returned in %ums.\n", wrap_accounted_delta((uint32_t) micros(), ping_req_time));
            ping_req_time = 0;
            ping_nonce    = 0;
          }
        }
      }
      else if (0 == strcmp("IMG_CAST", fxn_name)) {
        // Counterparty is sending us an image.
        //dump_msg_debug = false;
      }
      else if (0 == strcmp("WHO", fxn_name)) {
        // Counterparty wants to know who we are.
        log.concatf("\tTODO: Unimplemented fxn: \n", fxn_name);
      }
      else {
        log.concatf("\tUnhandled fxn: \n", fxn_name);
      }
    }
    else {
      log.concat("\tRX'd message with no specified fxn.\n");
    }
  }
  else {
    if (msg->isReply()) {
      log.concatf("\tRX'd ACK for msg %u.\n", msg->uniqueId());
      dump_msg_debug = false;
    }
    else {
      log.concat("\tRX'd message with no payload.\n");
    }
  }

  if (dump_msg_debug) {
    log.concat('\n');
    msg->printDebug(&log);
  }

  if (msg->expectsReply()) {
    int8_t ack_ret = msg->ack();
    log.concatf("\n\tACK'ing %u returns %d.\n", msg->uniqueId(), ack_ret);
  }
  c3p_log(LOG_LEV_INFO, __PRETTY_FUNCTION__, &log);
}


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


/*
* TODO: Locally pre-empted console handler to facilitate `ping`, which is no
*   longer required. Reduce into a shunt.
*/
int callback_link_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (mlink_local) {
    char* cmd = args->position_trimmed(0);
    // We interdict if the command is something specific to this application.
    if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
      // Send a description request message.
      ping_req_time = (uint32_t) millis();
      ping_nonce = randomUInt32();
      KeyValuePair* a = new KeyValuePair("PING",  "fxn");
      a->append(ping_req_time, "time_ms");
      a->append(ping_nonce,    "rand");
      int8_t ret_local = mlink_local->send(a, true);
      text_return->concatf("Ping send() returns ID %u\n", ret_local);
      ret = 0;
    }
    else ret = mlink_local->console_handler(text_return, args);

  }
  else {
    text_return->concat("mlink_local is not allocated.\n");
  }

  return ret;
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
    bool should_sleep = true;
    uint32_t millis_now = millis();

    if (0 < console_uart.poll()) {
      should_sleep = false;
    }
    if (0 < movi_uart.poll()) {
      should_sleep = false;
    }
    movi.poll();

    if (mlink_local) {
      StringBuilder link_log;
      mlink_local->poll(&link_log);
      if (!link_log.isEmpty()) {
        c3p_log(LOG_LEV_INFO, TAG, &link_log);
      }
    }

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

  // The unit has a bi-color LED with a common anode.
  // Start with the LED off.
  pinMode(LED_R_PIN,  GPIOMode::ANALOG_OUT);
  pinMode(LED_G_PIN,  GPIOMode::ANALOG_OUT);
  analogWrite(LED_G_PIN, 1.0);
  analogWrite(LED_R_PIN, 1.0);

  /* Start the console UART and attach it to the console. */
  console_uart.readCallback(&console);    // Attach the UART to console...
  console.setOutputTarget(&console_uart); // ...and console to UART.
  console.setTXTerminator(LineTerm::CRLF); // Best setting for "idf.py monitor"
  console.setRXTerminator(LineTerm::LF);   // Best setting for "idf.py monitor"
  console.setPromptString(console_prompt_str);
  console.emitPrompt(true);
  console.localEcho(true);
  console.printHelpOnFail(true);

  console.defineCommand("help",        '?',  "Prints help to console.", "[<specific command>]", 0, callback_help);
  console.defineCommand("console",     '\0', "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  platform.configureConsole(&console);
  console.defineCommand("link",        'l',  "Linked device tools.", "", 0, callback_link_tools);
  console.defineCommand("str",         '\0', "Storage tools", "", 0, console_callback_esp_storage);
  console.defineCommand("movi",        'm',  "MOVI tools", "", 0, console_callback_movi);
  console.defineCommand("uart",        'u',  "UART tools", "<adapter> [init|deinit|reset|poll]", 0, callback_uart_tools);

  console.init();

  StringBuilder ptc("Chatterbox ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");

  int ret = movi_uart.init(&uart2_opts);
  ptc.concatf("MOVI UART init() returns %d\n", ret);
  // The MOVI driver doesn't require a call to an init() function. It has an
  //   internal finite state machine that is smart enough to do things for
  //   itself, if given a working UART reference and regular polling.
  // It does, however, require us to setup callbacks to get most out of it.
  // TODO: That^^

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
