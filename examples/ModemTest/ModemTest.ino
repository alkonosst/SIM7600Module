/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

/** Explanation of the example:
 * - The modem is initialized on setup and some callbacks are set for TCP and MQTT events.
 * - The modem loop function is called to process URCs.
 * - The handleSerial function in loop reads the serial input and performs the action described on
 * each letter (take a look at the switch case in handleSerial function).
 * - This example is tested with a SIM7600G-H-M.2 module connected to an ESP32 via UART. Modify
 *   according to your setup.
 */

#include "SIM7600MQTTClient.h"
#include "SIM7600TCPClient.h"

// Modem configuration
#define GSM_SERIAL Serial1
const uint8_t GSM_GPIO_ENABLE       = 40;
const uint8_t GSM_UART_RX           = 41;
const uint8_t GSM_UART_TX           = 42;
const uint16_t GSM_UART_BUFFER_SIZE = 1024;
const char* GSM_APN                 = "internet";
const char* GSM_USER                = "";
const char* GSM_PASS                = "";

// TCP configuration
const char* TCP_SERVER        = "api.ipify.org";
const uint16_t TCP_PORT       = 80;
const char TCP_HTTP_GET_REQ[] = "GET / HTTP/1.0\r\n"
                                "Host: api.ipify.org\r\n"
                                "Connection: close\r\n"
                                "\r\n";

// MQTT configuration
const char* MQTT_SERVER       = "broker.emqx.io";
const char* MQTT_TOPIC        = "sim7600/test";
const char* MQTT_WILL_TOPIC   = "sim7600/will";
const char* MQTT_WILL_MESSAGE = "SIM7600 disconnected!";

// Global objects
SIM7600::Modem modem(&Serial1);
SIM7600::TCPClient tcp(&modem);
SIM7600::MQTTClient mqtt(&modem);

// Keyboard handling to do tests
void handleSerial();

/* Callback functions definitions */
// Modem
void modemReadyCB() { Serial.println("Event: Modem is ready!"); }

void networkChangedCB(const bool registered, const SIM7600::RegStatus reg_status) {
  Serial.printf("Event: Network changed! Registered: %s, Status: %u\r\n",
    registered ? "Yes" : "No",
    static_cast<uint8_t>(reg_status));
}

// TCP
void tcpNetworkClosedCB() { Serial.println("Event: TCP network connection closed!"); }

void tcpDataReceivedCB(SIM7600::TCPClient* const client) {
  Serial.printf("Event: Data received in TCPClient id: %u\r\n", client->getClientID());
}

void tcpConnectionClosedCB(SIM7600::TCPClient* const client, const SIM7600::TCPCloseReason reason) {
  Serial.printf("Event: Connection closed in TCPClient id: %u, reason: %u\r\n",
    client->getClientID(),
    static_cast<uint8_t>(reason));
}

// MQTT
void mqttNetworkClosedCB() { Serial.println("Event: MQTT network connection closed!"); }

void mqttConnectionLostCB(SIM7600::MQTTClient* const client,
  const SIM7600::MQTTConnLostCause cause) {
  Serial.printf("Event: MQTT connection lost in MQTTClient id: %u, cause: %u\r\n",
    client->getClientID(),
    static_cast<uint8_t>(cause));
}

void mqttPingFailedCB(SIM7600::MQTTClient* const client) {
  Serial.printf("Event: MQTT ping failed in MQTTClient id: %u\r\n", client->getClientID());
}

void mqttRxStartCB(SIM7600::MQTTClient* const client, const size_t topic_length,
  const size_t payload_length) {
  Serial.printf("Event: MQTT message received start in MQTTClient id: %u, topic length: %u, "
                "payload length: %u\r\n",
    client->getClientID(),
    topic_length,
    payload_length);
}

void mqttRxTopicCB(SIM7600::MQTTClient* const client, const char* topic,
  const size_t topic_length) {
  Serial.printf(
    "Event: MQTT message topic received in MQTTClient id: %u, topic: %s, length: %u\r\n",
    client->getClientID(),
    topic,
    topic_length);
}

void mqttRxPayloadCB(SIM7600::MQTTClient* const client, const uint8_t* payload,
  const size_t payload_length) {
  Serial.printf(
    "Event: MQTT message payload received in MQTTClient id: %u, payload: %s, length: %u\r\n",
    client->getClientID(),
    payload,
    payload_length);
}

void mqttRxEndCB(SIM7600::MQTTClient* const client) {
  Serial.printf("Event: MQTT message received end in MQTTClient id: %u\r\n", client->getClientID());
}
/* ---------------------------------------------------------------------------------------------- */

void setup() {
  // Give some time for Serial to initialize
  delay(2000);

  Serial.begin(115200);

  // Initialize GSM Serial
  GSM_SERIAL.setRxBufferSize(GSM_UART_BUFFER_SIZE);
  GSM_SERIAL.setTxBufferSize(GSM_UART_BUFFER_SIZE);
  GSM_SERIAL.begin(115200, SERIAL_8N1, GSM_UART_RX, GSM_UART_TX);

  // Enable modem power (in the case of SIM7600 M.2 version)
  pinMode(GSM_GPIO_ENABLE, OUTPUT);
  digitalWrite(GSM_GPIO_ENABLE, HIGH);

  // Modem callbacks
  modem.setModemReadyCallback(modemReadyCB);
  modem.setNetworkChangedCallback(networkChangedCB);
  modem.setTCPNetworkClosedCallback(tcpNetworkClosedCB);
  modem.setMQTTNetworkClosedCallback(mqttNetworkClosedCB);

  // TCP callbacks
  tcp.setDataReceivedCallback(tcpDataReceivedCB);
  tcp.setConnectionClosedCallback(tcpConnectionClosedCB);

  // MQTT callbacks
  mqtt.setConnectionLostCallback(mqttConnectionLostCB);
  mqtt.setPingFailedCallback(mqttPingFailedCB);
  mqtt.setRxStartCallback(mqttRxStartCB);
  mqtt.setRxTopicCallback(mqttRxTopicCB);
  mqtt.setRxPayloadCallback(mqttRxPayloadCB);
  mqtt.setRxEndCallback(mqttRxEndCB);

  while (true) {
    Serial.println("Initializing modem...");
    SIM7600::Status status = modem.init();

    if (status == SIM7600::Status::Success) {
      Serial.println("Modem initialized successfully!");
      break;
    } else {
      Serial.printf("Modem initialization failed: %s\r\n", SIM7600::statusToString(status));
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void loop() {
  // Handle serial input
  handleSerial();

  // Let the modem process URCs (Unsolicited Result Codes)
  modem.loop();
}

void handleSerial() {
  if (!Serial.available()) return;

  char c = Serial.read();

  // Ignore \r
  if (c == '\r') return;

  if (c == '\n') {
    Serial.println("> ");
  } else {
    Serial.printf("> %c\r\n", c);
  }

  SIM7600::Status status;

  switch (c) {
    // ESP32 and modem control
    case '|':
    {
      Serial.println("Restarting ESP32...");
      ESP.restart();
    } break;

    case '/':
    {
      status = modem.init();

      if (status == SIM7600::Status::Success) {
        Serial.println("Modem initialized successfully!");
      } else {
        Serial.printf("Modem initialization failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '*':
    {
      status = modem.reset();

      if (status == SIM7600::Status::Success) {
        Serial.println("Modem reset successfully");
      } else {
        Serial.printf("Modem reset failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '-':
    {
      status = modem.powerOff();

      if (status == SIM7600::Status::Success) {
        Serial.println("Modem powered off successfully");
      } else {
        Serial.printf("Modem power off failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    // Modem tests
    case '1':
    {
      float rssi, ber_min, ber_max;

      status = modem.getSignalQuality(rssi, ber_min, ber_max);

      if (status == SIM7600::Status::Success) {
        Serial.printf("Signal Quality - RSSI: %.1f, BER Min: %.1f, BER Max: %.1f\r\n",
          rssi,
          ber_min,
          ber_max);
      } else {
        Serial.printf("Getting signal quality failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '2':
    {
      status = modem.configureAPN(GSM_APN, GSM_USER, GSM_PASS);

      if (status == SIM7600::Status::Success) {
        Serial.println("APN configured successfully");
      } else {
        Serial.printf("APN configuration failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '3':
    {
      SIM7600::RegStatus reg_status;
      status = modem.waitForNetworkRegistration(reg_status);

      if (status == SIM7600::Status::Success) {
        Serial.printf("Registered to network successfully: %u\r\n",
          static_cast<uint8_t>(reg_status));
      } else {
        Serial.printf("Network registration failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '4':
    {
      Serial.printf("Currently registered on network: %s, Status: %u\r\n",
        modem.isCurrentlyRegisteredOnNetwork() ? "Yes" : "No",
        static_cast<uint8_t>(modem.getCurrentRegistrationStatus()));
    } break;

    case '5':
    {
      status = modem.setNTPServer("pool.ntp.org", 0);

      if (status == SIM7600::Status::Success) {
        Serial.printf("NTP server set successfully\r\n");
      } else {
        Serial.printf("NTP server setup failed: %s\r\n", SIM7600::statusToString(status));
      }

      SIM7600::NTPSyncStatus ntp_status;
      status = modem.synchronizeTime(ntp_status);
      if (status == SIM7600::Status::Success) {
        Serial.printf("Time synchronized successfully, NTP Status: %u\r\n",
          static_cast<uint8_t>(ntp_status));
      } else {
        Serial.printf("Time synchronization failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '6':
    {
      SIM7600::NTPTimeData time_data;

      status = modem.getNetworkTime(time_data);

      if (status == SIM7600::Status::Success) {
        Serial.printf("Network time: %04u-%02u-%02u %02u:%02u:%02u TZ: %d\r\n",
          time_data.year,
          time_data.month,
          time_data.day,
          time_data.hour,
          time_data.minute,
          time_data.second,
          time_data.time_zone);
      } else {
        Serial.printf("Getting network time failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case '7':
    {
      status = modem.setGPSAntennaVoltage(3050);

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS antenna voltage set successfully");
      } else {
        Serial.printf("Setting GPS antenna voltage failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case '8':
    {
      uint16_t voltage_mv;
      status = modem.getGPSAntennaVoltage(voltage_mv);

      if (status == SIM7600::Status::Success) {
        Serial.printf("GPS antenna voltage: %u mV\r\n", voltage_mv);
      } else {
        Serial.printf("Getting GPS antenna voltage failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case '9':
    {
      status = modem.enableGPSAntennaVoltage();

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS antenna voltage enabled successfully");
      } else {
        Serial.printf("Enabling GPS antenna voltage failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case '0':
    {
      status = modem.disableGPSAntennaVoltage();

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS antenna voltage disabled successfully");
      } else {
        Serial.printf("Disabling GPS antenna voltage failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case 'q':
    {
      bool enabled;
      status = modem.isGPSAntennaVoltageEnabled(enabled);

      if (status == SIM7600::Status::Success) {
        Serial.printf("GPS antenna voltage enabled: %s\r\n", enabled ? "Yes" : "No");
      } else {
        Serial.printf("Checking if GPS antenna voltage is enabled failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case 'w':
    {
      status = modem.enableGPS();

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS enabled successfully");
      } else {
        Serial.printf("Enabling GPS failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'e':
    {
      status = modem.disableGPS();

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS disabled successfully");
      } else {
        Serial.printf("Disabling GPS failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'r':
    {
      bool enabled;
      status = modem.isGPSEnabled(enabled);

      if (status == SIM7600::Status::Success) {
        Serial.printf("GPS enabled: %s\r\n", enabled ? "Yes" : "No");
      } else {
        Serial.printf("Checking if GPS is enabled failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 't':
    {
      status = modem.enableGPSAutoStart(true);

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS auto start enabled successfully");
      } else {
        Serial.printf("Enabling GPS auto start failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'y':
    {
      status = modem.enableGPSAutoStart(false);

      if (status == SIM7600::Status::Success) {
        Serial.println("GPS auto start disabled successfully");
      } else {
        Serial.printf("Disabling GPS auto start failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'u':
    {
      bool enabled;
      status = modem.getGPSAutoStart(enabled);

      if (status == SIM7600::Status::Success) {
        Serial.printf("GPS auto start enabled: %s\r\n", enabled ? "Yes" : "No");
      } else {
        Serial.printf("Getting GPS auto start status failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case 'i':
    {
      static SIM7600::GPSData gps_data;
      status = modem.getGPSData(gps_data);

      if (status == SIM7600::Status::Success) {
        Serial.printf(
          "GPS Data:\r\n-> Fix Status: %u, Lat: %.6lf, Lon: %.6lf\r\n-> Date: %02u-%02u-%04u, "
          "Time: "
          "%02u:%02u:%02u\r\n-> Speed: %.2f, Course: %.2f, Altitude: %.2f\r\n-> GPS: %u, GLONASS: "
          "%u, BeiDou: "
          "%u\r\n-> PDOP: %.2f, HDOP: %.2f, VDOP: %.2f\r\n",
          gps_data.fix_status,
          gps_data.latitude,
          gps_data.longitude,
          gps_data.day,
          gps_data.month,
          gps_data.year,
          gps_data.hour,
          gps_data.minute,
          gps_data.second,
          gps_data.speed,
          gps_data.course,
          gps_data.altitude,
          gps_data.gps_satellites,
          gps_data.glonass_satellites,
          gps_data.beidou_satellites,
          gps_data.pdop,
          gps_data.hdop,
          gps_data.vdop);

      } else {
        Serial.printf("Getting GPS data failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    // TCP Client tests
    case 'o':
    {
      status = modem.startTCPIPService();

      if (status == SIM7600::Status::Success) {
        Serial.println("TCP/IP service started successfully");
      } else {
        Serial.printf("Starting TCP/IP service failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'p':
    {
      status = modem.stopTCPIPService();

      if (status == SIM7600::Status::Success) {
        Serial.println("TCP/IP service stopped successfully");
      } else {
        Serial.printf("Stopping TCP/IP service failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'a':
    {
      status = tcp.connectToHost(TCP_SERVER, TCP_PORT);

      if (status == SIM7600::Status::Success) {
        Serial.println("TCP connected successfully");
      } else {
        Serial.printf("TCP connection failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 's':
    {
      status = tcp.disconnect();

      if (status == SIM7600::Status::Success) {
        Serial.println("TCP disconnected successfully");
      } else {
        Serial.printf("TCP disconnection failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'd':
    {
      bool connected;
      status = tcp.isConnected(connected);

      if (status == SIM7600::Status::Success) {
        Serial.printf("TCP connected: %s\r\n", connected ? "Yes" : "No");
      } else {
        Serial.printf("Checking TCP connection failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'f':
    {
      // Don't send the null terminator
      static const size_t request_size = sizeof(TCP_HTTP_GET_REQ) - 1;

      size_t bytes_sent = 0;

      status =
        tcp.sendData(reinterpret_cast<const uint8_t*>(TCP_HTTP_GET_REQ), request_size, bytes_sent);

      if (status == SIM7600::Status::Success) {
        Serial.printf("TCP data sent successfully: %u bytes\r\n", bytes_sent);
      } else {
        Serial.printf("TCP data send failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'g':
    {
      size_t available = 0;
      status           = tcp.getAvailableBytes(available);

      if (status == SIM7600::Status::Success) {
        Serial.printf("TCP available bytes to read: %u\r\n", available);
      } else {
        Serial.printf("Getting available TCP bytes failed: %s\r\n",
          SIM7600::statusToString(status));
      }
    } break;

    case 'h':
    {
      static char rx_buffer[512];
      size_t bytes_read = 0;

      // Leave space for null terminator
      status =
        tcp.readData(reinterpret_cast<uint8_t*>(rx_buffer), sizeof(rx_buffer) - 1, bytes_read);

      if (status == SIM7600::Status::Success) {
        rx_buffer[bytes_read] = '\0';
        Serial.printf("TCP data read successfully: %u byte\r\n", bytes_read);
        Serial.println("---> Content: <---");
        Serial.println(rx_buffer);
        Serial.println("---> End of content <---");
      }

      else if (status == SIM7600::Status::TCPNoDataAvailable) {
        Serial.println("No TCP data available to read");
      }

      else {
        Serial.printf("TCP data read failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    // MQTT Client tests
    case 'j':
    {
      status = modem.startMQTTService();

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT service started successfully");
      } else {
        Serial.printf("Starting MQTT service failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'k':
    {
      status = modem.stopMQTTService();

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT service stopped successfully");
      } else {
        Serial.printf("Stopping MQTT service failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'l':
    {
      status = mqtt.acquireClient("sim7600", false, SIM7600::MQTTVersion::V3_1_1);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT client acquired successfully");
      } else {
        Serial.printf("Acquiring MQTT client failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'z':
    {
      status = mqtt.releaseClient();

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT client released successfully");
      } else {
        Serial.printf("Releasing MQTT client failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'x':
    {
      status =
        mqtt.setLastWillMessage(MQTT_WILL_TOPIC, MQTT_WILL_MESSAGE, SIM7600::MQTTQoS::AtLeastOnce);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT will set successfully");
      } else {
        Serial.printf("Setting MQTT will failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'c':
    {
      status = mqtt.connect(MQTT_SERVER);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT connected successfully");
      } else {
        Serial.printf("MQTT connection failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'v':
    {
      status = mqtt.disconnect();

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT disconnected successfully");
      } else {
        Serial.printf("MQTT disconnection failed: %s", SIM7600::statusToString(status));
      }
    } break;

    case 'b':
    {
      bool connected;
      status = mqtt.isConnected(connected);

      if (status == SIM7600::Status::Success) {
        Serial.printf("MQTT connected: %s\r\n", connected ? "Yes" : "No");
      } else {
        Serial.printf("Checking MQTT connection failed: %s\r\n", SIM7600::statusToString(status));
      }
    } break;

    case 'n':
    {
      status = mqtt.subscribe(MQTT_TOPIC, SIM7600::MQTTQoS::AtLeastOnce);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT subscribed to topic successfully");
      } else {
        Serial.printf("Subscribing to MQTT topic failed: %s\r\n", SIM7600::statusToString(status));
        break;
      }
    } break;

    case 'm':
    {
      status = mqtt.unsubscribe(MQTT_TOPIC);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT unsubscribed from topic successfully");
      } else {
        Serial.printf("Unsubscribing from MQTT topic failed: %s\r\n",
          SIM7600::statusToString(status));
        break;
      }
    } break;

    case ',':
    {
      static char mqtt_payload[16];

      uint32_t rand = random(0, 100);
      snprintf(mqtt_payload, sizeof(mqtt_payload), "%u", rand);

      status = mqtt.publish(MQTT_TOPIC,
        reinterpret_cast<const uint8_t*>(mqtt_payload),
        strlen(mqtt_payload),
        SIM7600::MQTTQoS::AtLeastOnce);

      if (status == SIM7600::Status::Success) {
        Serial.println("MQTT message published successfully");
      } else {
        Serial.printf("Publishing MQTT message failed: %s\r\n", SIM7600::statusToString(status));
        break;
      }
    } break;
  }
}
