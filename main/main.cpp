#include <math.h>

/* ManuvrDrivers */
#include <ManuvrDrivers.h>

/* CppPotpourri */
#include <ESP32.h>
#include <UARTAdapter.h>
#include <ParsingConsole.h>
#include <I2CAdapter.h>
#include <SPIAdapter.h>
#include <GPSWrapper.h>

/* Local includes */
#include "NullamLucet.h"


#ifdef __cplusplus
extern "C" {
#endif


/*
Pins
-------------
IO00  RMII CLK
IO02              // Pigtail soldered to SD socket
IO04              // Pigtail soldered to SD socket
IO05
IO12              // Pigtail soldered to SD socket
IO13  SX1503_IRQ <Non-boot>  // Pigtail soldered to SD socket
IO14  SDA  <Non-boot>  // Pigtail soldered to SD socket
IO15  SCL              // Pigtail soldered to SD socket
IO16  SPI1_CLK
IO17  LORA_CS
IO18  RMII MDIO
IO19  RMII TXD0
IO21  RMII TX_EN
IO22  RMII TXD1
IO23  RMII MDC
IO25  RMII RXD0
IO26  RMII RXD1
IO27  RMII RX_CRS_DV
IO32  SPI1_MOSI
IO33  LORA_RESET
IO34  LORA_DIO2
IO35  LORA_DIO1
IO36  LORA_DIO0
IO39  SPI1_MISO
*/

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp32/rom/ets_sys.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_eth.h"

//#include "esp_ota_ops.h"
//#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"


#define CONFIG_PHY_ADDRESS         0
#define CONFIG_PHY_SMI_MDC_PIN    23
#define CONFIG_PHY_SMI_MDIO_PIN   18

#define PIN_SMI_MDC   CONFIG_PHY_SMI_MDC_PIN
#define PIN_SMI_MDIO  CONFIG_PHY_SMI_MDIO_PIN

/* Platform pins */
#define SDA_PIN             14  // IO14
#define SCL_PIN             15  // IO15
#define SPI_CLK_PIN         16  // IO16
#define LORA_CS_PIN         17  // IO17
#define SPI_MOSI_PIN        32  // IO32
#define LORA_RESET_PIN      33  // IO33
#define LORA_DIO2_PIN       34  // IO34
#define LORA_DIO1_PIN       35  // IO35
#define LORA_DIO0_PIN       36  // IO36
#define SPI_MISO_PIN        39  // IO39



/* OTA parameters that probably ought to be imparted at provisioning. */
#define EXAMPLE_SERVER_URL "ian-app.home.joshianlindsay.com"
//extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


TMP102* tmp102                  = nullptr;
esp_mqtt_client_handle_t client = nullptr;

static bool connected_have_ip   = false;
static bool connected_mqtt      = false;


const I2CAdapterOptions i2c0_opts(
  0,   // Device number
  14,  // (sda)
  15,  // (scl)
  0,   // No pullups.
  100000
);

const SX1276Opts sx1276_opts(
  LORA_RESET_PIN,  // Reset
  LORA_CS_PIN,     // CS
  LORA_DIO0_PIN,   // D0
  LORA_DIO1_PIN,   // D1
  LORA_DIO2_PIN,   // D2
  LORABand::BAND_915
);


UARTOpts uart1_opts {
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


static const char* TAG          = "main-cpp";
ParsingConsole console(128);
ESP32StdIO console_uart;
UARTAdapter gps_uart(1, 13, 12, 255, 255, 256, 256);
SPIAdapter spi_bus(1, SPI_CLK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, 8);
I2CAdapter i2c0(&i2c0_opts);
GPSWrapper gps;

NullamLucetOpts relay_opts {};

esp_eth_phy_t* phy = nullptr;


static void eth_gpio_config_rmii() {
  phy_rmii_configure_data_interface_pins();
  phy_rmii_smi_configure_pins(PIN_SMI_MDC, PIN_SMI_MDIO);
}


/**
 * @brief event handler for ethernet
 *
 * @param ctx
 * @param event
 * @return esp_err_t
 */
static esp_err_t eth_event_handler(void* ctx, system_event_t* event) {
  tcpip_adapter_ip_info_t ip;

  switch (event->event_id) {
    case SYSTEM_EVENT_ETH_CONNECTED:
      ESP_LOGI(TAG, "Ethernet Link Up");
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      ESP_LOGI(TAG, "Ethernet Link Down");
      connected_have_ip = false;
      break;
    case SYSTEM_EVENT_ETH_START:
      ESP_LOGI(TAG, "Ethernet Started");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
      ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_ETH, &ip));
      ESP_LOGI(TAG, "Ethernet Got IP Addr");
      ESP_LOGI(TAG, "~~~~~~~~~~~");
      ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip.ip));
      ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip.netmask));
      ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip.gw));
      ESP_LOGI(TAG, "~~~~~~~~~~~");
      connected_have_ip = true;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      ESP_LOGI(TAG, "Ethernet Stopped");
      connected_have_ip = false;
      break;
    default:
      ESP_LOGI(TAG, "Unhandled event: %d\n", event->event_id);
      break;
  }
  return ESP_OK;
}


static int proc_mqtt_payload(const char* topic, uint8_t* buf, unsigned int len) {
  const char* tok1 = (const char*) topic;
  if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ble-ping")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 5);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ble-command")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 2);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/lora-message")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 4);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/lora-command")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 3);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/conf")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 1);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }
  else if (0 == StringBuilder::strcasestr(tok1, "/RadioRelay/ota-update")) {
    int num_val = atoi((const char*) buf);
    //ManuvrMsg* event = Kernel::returnEvent(MANUVR_MSG_DIMMER_SET_LEVEL);
    //event->addArg((uint8_t) 2);
    //event->addArg((uint8_t) num_val);
    //Kernel::staticRaiseEvent(event);
  }

  return 0;
}


/**
 * @brief event handler for MQTT
 *
 * @param ctx
 * @param event
 * @return esp_err_t
 */
static int mqtt_event_handler(esp_mqtt_event_t* event) {
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  // your_context_t *context = event->context;
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      connected_mqtt = true;
      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit2", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit5", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit4", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit3", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit1", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/circuit0", 0);
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

      msg_id = esp_mqtt_client_subscribe(client, "/theta/color", 0);
      //msg_id = esp_mqtt_client_subscribe(client, "#", 0);

      //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
      //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_DISCONNECTED:
      connected_mqtt = false;
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      break;

    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_DATA:
      {
        char* safe_topic   = (char*) alloca(event->topic_len + 1);
        uint8_t* safe_data = (uint8_t*) alloca(event->data_len + 1);
        memcpy(safe_topic, event->topic, event->topic_len);
        memcpy(safe_data, event->data, event->data_len);
        *(safe_topic + event->topic_len) = 0;
        *(safe_data + event->data_len)   = 0;
        printf("TOPIC0: %s\n", safe_topic);
        printf("DATA=%.*s\r\n", event->data_len, safe_data);
        proc_mqtt_payload(safe_topic, safe_data, event->data_len);
      }
      break;

    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      break;
    default:
      ESP_LOGI(TAG, "MQTT_EVENT Default case");
      break;
  }
  return ESP_OK;
}


void mqtt_send_temperature() {
  //StringBuilder _log;
  //gpsptr->printDebug(&_log);
  if (connected_mqtt && (nullptr != tmp102)) {
    StringBuilder dat;
    //tmp102->issue_value_map(TCode::CBOR, &dat);
    // SensorError err = tmp102->readAsString(0, &dat);
    // if (SensorError::NO_ERROR == err) {
    //   printf("temperature (%s).\n", (const char*) dat.string());
    //   int msg_id = esp_mqtt_client_publish(client, "/theta/radio-relay/temperature", (const char*) dat.string(), dat.length(), 0, 0);
    // }
    // else {
    //   printf("Failed to get temperature (%s).\n", SensorWrapper::errorString(err));
    // }
  }
  //Kernel::log(&_log);
}



void manuvr_task(void* pvParameter) {
  SX1276 lora(&sx1276_opts);
  SX1503 sx1503(13, 255);
  NullamLucet nl(&relay_opts);

  tmp102 = new TMP102(0x49);

  esp_mqtt_client_config_t mqtt_cfg;
  memset((void*) &mqtt_cfg, 0, sizeof(mqtt_cfg));
  mqtt_cfg.uri  = "mqtt://" EXAMPLE_SERVER_URL;
  mqtt_cfg.port = 1883;
  mqtt_cfg.event_handle = mqtt_event_handler;
  //mqtt_cfg.user_context = (void *)your_context

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_start(client);


  while (1) {
    bool should_sleep = true;
    while (0 < spi_bus.poll()) {
      should_sleep = false;
    }
    while (0 < spi_bus.service_callback_queue()) {
      should_sleep = false;
    }
    while (0 < i2c0.poll()) {
      should_sleep = false;
    }

    gps_uart.poll();
    //nl.poll();
    //nl.fetchLog(&_log);

    if (0 < _log.length()) {
      // Drain any accumulated log into the console output.
      console_uart.provideBuffer(&_log);
      should_sleep = false;
    }
    if (0 < console_uart.poll()) {
      should_sleep = false;
    }

    if (should_sleep) {
      vTaskDelay(1);
    }
  }
}


/*******************************************************************************
* OTA functions. Based on esp-idf example for native API use.
*******************************************************************************/
//
// static void http_cleanup(esp_http_client_handle_t client) {
//   esp_http_client_close(client);
//   esp_http_client_cleanup(client);
// }
//
//
// static void __attribute__((noreturn)) task_fatal_error() {
//   ESP_LOGE(TAG, "Exiting task due to fatal error...");
//   (void)vTaskDelete(NULL);
//   while (1) {}
// }
//
//
// static void ota_task(void *pvParameter) {
//   esp_err_t err;
//   char ota_write_data[1024 + 1] = { 0 };
//
//   /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
//   esp_ota_handle_t update_handle = 0;
//   const esp_partition_t *update_partition = NULL;
//   ESP_LOGI(TAG, "Starting OTA service thread...");
//
//   const esp_partition_t* configured = esp_ota_get_boot_partition();
//   const esp_partition_t* running = esp_ota_get_running_partition();
//
//     if (configured != running) {
//       ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x", configured->address, running->address);
//       ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
//     }
//     ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)", running->type, running->subtype, running->address);
//
//     /* Wait for the callback to set the CONNECTED_BIT in the event group. */
//     while (!connected_have_ip) {
//       vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
//     ESP_LOGI(TAG, "Checking for firmware update...");
//
//     esp_http_client_config_t config;
//     memset((void*) &config, 0, sizeof(config));
//     config.url = "https://" EXAMPLE_SERVER_URL "/";
//     config.cert_pem = (char *)server_cert_pem_start;
//
//
//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL) {
//         ESP_LOGE(TAG, "Failed to initialise HTTP connection");
//         task_fatal_error();
//     }
//     err = esp_http_client_open(client, 0);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
//         esp_http_client_cleanup(client);
//         task_fatal_error();
//     }
//     esp_http_client_fetch_headers(client);
//
//     update_partition = esp_ota_get_next_update_partition(NULL);
//     ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
//     assert(update_partition != NULL);
//
//     err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
//         http_cleanup(client);
//         task_fatal_error();
//     }
//     ESP_LOGI(TAG, "esp_ota_begin succeeded");
//
//     int binary_file_length = 0;
//     /*deal with all receive packet*/
//     while (1) {
//       int data_read = esp_http_client_read(client, ota_write_data, sizeof(ota_write_data)-1);
//       if (data_read < 0) {
//         ESP_LOGE(TAG, "Error: SSL data read error");
//         http_cleanup(client);
//         task_fatal_error();
//       }
//       else if (data_read > 0) {
//         err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
//         if (err != ESP_OK) {
//           http_cleanup(client);
//           ESP_LOGE(TAG, "Failed to write OTA image: %s", esp_err_to_name(err));
//           task_fatal_error();
//         }
//         binary_file_length += data_read;
//         ESP_LOGD(TAG, "Written image length %d", binary_file_length);
//       }
//       else if (data_read == 0) {
//         ESP_LOGI(TAG, "Connection closed,all data received");
//         break;
//       }
//     }
//     ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);
//
//     if (esp_ota_end(update_handle) != ESP_OK) {
//         ESP_LOGE(TAG, "esp_ota_end failed!");
//         http_cleanup(client);
//         task_fatal_error();
//     }
//
//     if (esp_partition_check_identity(esp_ota_get_running_partition(), update_partition) == true) {
//         ESP_LOGI(TAG, "The current running firmware is same as the firmware just downloaded");
//         int i = 0;
//         ESP_LOGI(TAG, "When a new firmware is available on the server, press the reset button to download it");
//         while(1) {
//           ESP_LOGI(TAG, "Waiting for a new firmware ... %d", ++i);
//           vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//     }
//
//   err = esp_ota_set_boot_partition(update_partition);
//   if (err != ESP_OK) {
//     ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
//     http_cleanup(client);
//     task_fatal_error();
//   }
//   ESP_LOGI(TAG, "Prepare to restart system!");
//   esp_restart();
// }
//
//
// void print_sha256(const uint8_t* image_hash, const char* label) {
//   char hash_print[65];
//   hash_print[64] = 0;
//   for (int i = 0; i < 32; ++i) {
//     sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
//   }
//   ESP_LOGI(TAG, "%s: %s", label, hash_print);
// }


/*******************************************************************************
* Console callbacks
*******************************************************************************/

int callback_help(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    console.printHelp(text_return, args->position_trimmed(0));
  }
  else {
    console.printHelp(text_return);
  }
  return 0;
}


int callback_print_history(StringBuilder* text_return, StringBuilder* args) {
  console.printHistory(text_return);
  return 0;
}



/*******************************************************************************
* Main function                                                                *
*******************************************************************************/
void app_main() {
  /*
  * The platform object is created on the stack, but takes no action upon
  *   construction. The first thing that should be done is to call the preinit
  *   function to setup the defaults of the platform.
  */
  platform_init();

    // uint8_t sha_256[32] = { 0 };
    // esp_partition_t partition;
    //
    // // get sha256 digest for the partition table
    // partition.address   = ESP_PARTITION_TABLE_OFFSET;
    // partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    // partition.type      = ESP_PARTITION_TYPE_DATA;
    // esp_partition_get_sha256(&partition, sha_256);
    // print_sha256(sha_256, "SHA-256 for the partition table: ");
    //
    // // get sha256 digest for bootloader
    // partition.address   = ESP_BOOTLOADER_OFFSET;
    // partition.size      = ESP_PARTITION_TABLE_OFFSET;
    // partition.type      = ESP_PARTITION_TYPE_APP;
    // esp_partition_get_sha256(&partition, sha_256);
    // print_sha256(sha_256, "SHA-256 for bootloader: ");
    //
    // // get sha256 digest for running partition
    // //esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    // print_sha256(sha_256, "SHA-256 for current firmware: ");

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_event_loop_init(eth_event_handler, NULL));

  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = CONFIG_PHY_ADDRESS;
  phy_config.reset_gpio_num = CONFIG_EXAMPLE_ETH_PHY_RST_GPIO;
  phy = esp_eth_phy_new_lan8720(&phy_config);

  //eth_config_t config = DEFAULT_ETHERNET_PHY_CONFIG;
  ///* Set the PHY address in the example configuration */
  //config.phy_addr    = (eth_phy_base_t) CONFIG_PHY_ADDRESS;
  //config.gpio_config = eth_gpio_config_rmii;
  //config.tcpip_input = tcpip_adapter_eth_input;
  //ESP_ERROR_CHECK(esp_eth_init(&config));
  ESP_ERROR_CHECK(esp_eth_enable());


  /* Start the console UART and attach it to the console. */
  console_uart.readCallback(&console);
  console.setOutputTarget(&console_uart);
  console.setTXTerminator(LineTerm::CRLF); // Best setting for "idf.py monitor"
  console.setRXTerminator(LineTerm::LF);   // Best setting for "idf.py monitor"
  console.localEcho(true);
  console.printHelpOnFail(true);
  console.init();

  console.defineCommand("help",        '?', ParsingConsole::tcodes_str_1, "Prints help to console.", "", 0, callback_help);
  console.defineCommand("history",     ParsingConsole::tcodes_0, "Print command history.", "", 0, callback_print_history);

  platform.configureConsole(&console);

  spi_bus.init();
  i2c0.init();
  gps.init();
  gps_uart.readCallback(&gps);
  //gps_uart.init(&uart1_opts);

  xTaskCreate(manuvr_task, "_manuvr", 32768, NULL, (tskIDLE_PRIORITY + 2), NULL);
  //xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
}

#ifdef __cplusplus
}
#endif
