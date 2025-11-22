#include <Arduino.h>

#include "TB_SIM7600Client.h"

#include "Shared_Attribute_Update.h"
#include "ThingsBoard.h"

static const char* tag = "main";

/* ---------------------------------------------------------------------------------------------- */
// NOTE: ESP-IDF por defecto envía logs a UART0, por lo que se redirigen a la terminal USB para
// liberar la UART0 para otro objetivo.
/* ---------------------------------------------------------------------------------------------- */
// Usar mutex para evitar problemas de concurrencia (función puede ser llamada de múltiples tareas)
static SemaphoreHandle_t log_mutex = nullptr;

int redirectLogs(const char* str, va_list list) {
  if (log_mutex != nullptr) xSemaphoreTake(log_mutex, portMAX_DELAY);

  static char buffer[2048];
  int ret = vsnprintf(buffer, sizeof(buffer), str, list);
  Serial.write(buffer);

  if (log_mutex != nullptr) xSemaphoreGive(log_mutex);

  return ret;
}
/* ---------------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------------- */
SIM7600::Modem modem(&Serial1);
TB_SIM7600Client tb_client(modem);

static const uint8_t GSM_PIN               = 40;
static const uint8_t GSM_RX                = 41;
static const uint8_t GSM_TX                = 42;
static const uint16_t GSM_UART_BUFFER_SIZE = 1024;

static const char* SERVER = "mqtt.thingsboard.cloud";
static const char* TOKEN  = "1cvfhhih8g22krjs37tx";

void handleSerial();
void handleModem();

bool network_closed = false;

void mqttNetworkClosedCB() {
  ESP_LOGW(tag, "MQTT network connection closed!");
  network_closed = true;
}

void mqttConnectionLostCB(SIM7600::MQTTClient* const client,
  const SIM7600::MQTTConnLostCause cause) {
  ESP_LOGW(tag,
    "MQTT connection lost in MQTTClient id: %u, cause: %u",
    client->getClientID(),
    static_cast<uint8_t>(cause));
}

void mqttPingFailedCB(SIM7600::MQTTClient* const client) {
  ESP_LOGW(tag, "MQTT ping failed in MQTTClient id: %u", client->getClientID());
}

// API de Shared Attributes
Shared_Attribute_Update<Default_Subscriptions_Amount, 1> shared_update;

const std::array<IAPI_Implementation*, 1> tb_apis{&shared_update};

// Cliente Thingsboard
ThingsBoardSized<16> tb(tb_client, 512, 512, 1024, tb_apis);

void processSharedAttributeUpdate(const JsonObjectConst& data) {
  ESP_LOGI(tag, "Received shared attribute update:");

  for (const auto& attr : data) {
    ESP_LOGI(tag, "-> %s: %s", attr.key().c_str(), attr.value().as<const char*>());
  }
}
/* ---------------------------------------------------------------------------------------------- */

void setup() {
  // Crear semáforo para función redirectLogs
  log_mutex = xSemaphoreCreateMutex();

  // Retrasar inicio para dar tiempo a la terminal USB a inicializarse
  delay(2000);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial1.begin(115200);

  // Redirigir logs a la terminal USB y configurar nivel de logs
  esp_log_set_vprintf(redirectLogs);
  esp_log_level_set("*", esp_log_level_t::ESP_LOG_VERBOSE);
  esp_log_level_set("ARDUINO", esp_log_level_t::ESP_LOG_DEBUG);

  Serial1.end();
  Serial1.setRxBufferSize(GSM_UART_BUFFER_SIZE);
  Serial1.setTxBufferSize(GSM_UART_BUFFER_SIZE);
  Serial1.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  Serial1.flush(false);

  pinMode(GSM_PIN, OUTPUT);
  digitalWrite(GSM_PIN, HIGH);

  ESP_LOGI(tag, "Starting...");
  bool initialized = false;

  // Modem config
  modem.setMQTTNetworkClosedCallback(mqttNetworkClosedCB);

  // MQTT config
  tb_client.getClient().setConnectionLostCallback(mqttConnectionLostCB);
  tb_client.getClient().setPingFailedCallback(mqttPingFailedCB);
  // mqtt.setRxStartCallback(mqttRxStartCB);
  // mqtt.setRxTopicCallback(mqttRxTopicCB);
  // mqtt.setRxPayloadCallback(mqttRxPayloadCB);
  // mqtt.setRxEndCallback(mqttRxEndCB);

  SIM7600::Status status;

  while (!initialized) {
    ESP_LOGI(tag, "Initializing modem...");
    status = modem.init();

    if (status == SIM7600::Status::Success) {
      initialized = true;
      ESP_LOGI(tag, "Modem initialized successfully");
    }

    else {
      ESP_LOGE(tag, "Modem initialization failed: %s", modem.statusToString(status));
      ESP_LOGI(tag, "Retrying in 5 seconds...");
      delay(5000);
    }
  }

  status = modem.configureAPN("internet");

  if (status == SIM7600::Status::Success) {
    ESP_LOGI(tag, "APN configured successfully");
  } else {
    ESP_LOGE(tag, "APN configuration failed: %s", modem.statusToString(status));
  }
}

void loop() {
  handleSerial();
  handleModem();
}

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  // Ignore \r
  if (c == '\r') return;

  if (c == '\n') {
    ESP_LOGI(tag, "> ");
  } else {
    ESP_LOGI(tag, "> %c", c);
  }

  switch (c) {
    SIM7600::Status status;

    // Modem tests
    case '|':
    {
      ESP_LOGI(tag, "Restarting ESP32...");
      ESP.restart();
    } break;

    case '1':
    {
      SIM7600::RegStatus reg_status;
      status = modem.waitForNetworkRegistration(reg_status);

      if (status == SIM7600::Status::Success) {
        ESP_LOGI(tag, "Registered to network successfully: %u", static_cast<uint8_t>(reg_status));
      } else {
        ESP_LOGE(tag, "Network registration failed: %s", modem.statusToString(status));
      }
    } break;

    case '2':
    {
      status = modem.startMQTTService();

      if (status == SIM7600::Status::Success) {
        ESP_LOGI(tag, "MQTT service started successfully");
      } else {
        ESP_LOGE(tag, "Starting MQTT service failed: %s", modem.statusToString(status));
      }
    } break;

    case '3':
    {
      status = modem.stopMQTTService();

      if (status == SIM7600::Status::Success) {
        ESP_LOGI(tag, "MQTT service stopped successfully");
      } else {
        ESP_LOGE(tag, "Stopping MQTT service failed: %s", modem.statusToString(status));
      }
    } break;

    case '4':
    {
      status =
        tb_client.getClient().acquireClient("sim7600-max", false, SIM7600::MQTTVersion::V3_1_1);

      if (status == SIM7600::Status::Success) {
        ESP_LOGI(tag, "MQTT client acquired successfully");
      } else {
        ESP_LOGE(tag, "Acquiring MQTT client failed: %s", modem.statusToString(status));
      }
    } break;

    case '5':
    {
      if (!tb.connect(SERVER, TOKEN)) {
        ESP_LOGE(tag, "ThingsBoard MQTT connection failed");
        break;
      }

      ESP_LOGI(tag, "ThingsBoard MQTT connected successfully");
    } break;

    case '6':
    {
      tb.disconnect();
      ESP_LOGI(tag, "ThingsBoard MQTT disconnected");
    } break;

    case '7':
    {
      uint32_t temp = random(1, 100);

      if (!tb.sendTelemetryData("simtest-pub", temp)) {
        ESP_LOGE(tag, "Sending telemetry data failed");
        break;
      }

      ESP_LOGI(tag, "Telemetry data sent successfully: %u", temp);
    } break;

    case '8':
    {
      constexpr std::array<const char*, 1> attributes{"simtest-sub"};
      const Shared_Attribute_Callback<1> callback(&processSharedAttributeUpdate, attributes);

      if (!shared_update.Shared_Attributes_Subscribe(callback)) {
        ESP_LOGE(tag, "Subscribing to shared attribute updates failed");
        break;
      }

      ESP_LOGI(tag, "Subscribed to shared attribute updates successfully");
    } break;

    case '0':
    {
      if (!network_closed) {
        ESP_LOGI(tag, "MQTT network is still open, no reconnection needed");
        break;
      }

      network_closed = false;
      ESP_LOGI(tag, "Reconnecting MQTT due to network closure...");

      SIM7600::Status status = modem.startMQTTService();

      if (status != SIM7600::Status::Success) {
        ESP_LOGE(tag, "Starting MQTT service failed: %s", modem.statusToString(status));
        break;
      }

      status =
        tb_client.getClient().acquireClient("sim7600-max", false, SIM7600::MQTTVersion::V3_1_1);

      if (status != SIM7600::Status::Success) {
        ESP_LOGE(tag, "Acquiring MQTT client failed: %s", modem.statusToString(status));
      }

      if (!tb.connect(SERVER, TOKEN)) {
        ESP_LOGE(tag, "ThingsBoard MQTT connection failed");
        break;
      }

      ESP_LOGI(tag, "ThingsBoard MQTT reconnected successfully");
    } break;
  }
}

void handleModem() {
  // modem.loop();
  tb.loop();
}