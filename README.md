<h1 align="center">
  <a><img src=".img/logo.png" alt="Logo" width="300"></a>
  <br>
  SIM7600Module
</h1>

<p align="center">
  <b>Transfer your data over 4G LTE with the SIM7600 module!</b>
</p>

<p align="center">
  <a href="https://www.ardu-badge.com/SIM7600Module">
    <img src="https://www.ardu-badge.com/badge/SIM7600Module.svg?" alt="Arduino Library Badge">
  </a>
  <a href="https://registry.platformio.org/libraries/alkonosst/SIM7600Module">
    <img src="https://badges.registry.platformio.org/packages/alkonosst/library/SIM7600Module.svg" alt="PlatformIO Registry">
  </a>
  <br><br>
  <a href="https://ko-fi.com/alkonosst">
    <img src="https://ko-fi.com/img/githubbutton_sm.svg" alt="Ko-fi">
    </a>
</p>

---

# Table of contents <!-- omit in toc -->

- [Description](#description)
- [Usage](#usage)
  - [Adding library to Arduino IDE](#adding-library-to-arduino-ide)
  - [Adding library to platformio.ini (PlatformIO)](#adding-library-to-platformioini-platformio)
  - [Using the library](#using-the-library)
    - [Including the library](#including-the-library)
    - [Namespaces](#namespaces)
    - [Status codes](#status-codes)
    - [Creating an instance of the Modem and initializing it](#creating-an-instance-of-the-modem-and-initializing-it)
    - [Creating an instance of client classes](#creating-an-instance-of-client-classes)
    - [Macro configuration](#macro-configuration)
      - [Changing TX and RX buffer sizes](#changing-tx-and-rx-buffer-sizes)
      - [Enabling debug output](#enabling-debug-output)
  - [Modem features](#modem-features)
    - [Initialization and status](#initialization-and-status)
    - [Power management](#power-management)
    - [GPS management](#gps-management)
    - [Network management](#network-management)
    - [NTP time synchronization](#ntp-time-synchronization)
    - [Start/stop services](#startstop-services)
    - [Callbacks](#callbacks)
  - [TCPClient features](#tcpclient-features)
    - [Connecting to a TCP server](#connecting-to-a-tcp-server)
    - [Transferring data](#transferring-data)
    - [Callbacks](#callbacks-1)
  - [MQTTClient features](#mqttclient-features)
    - [Connecting to an MQTT broker](#connecting-to-an-mqtt-broker)
    - [Publishing and subscribing to topics](#publishing-and-subscribing-to-topics)
    - [Callbacks](#callbacks-2)
- [Troubleshooting](#troubleshooting)
  - [Module doesn't respond](#module-doesnt-respond)
  - [Can't register on network](#cant-register-on-network)
  - [GPS doesn't get a fix](#gps-doesnt-get-a-fix)
  - [MQTT/TCP connection fails](#mqtttcp-connection-fails)
  - [Enable verbose logging for debugging](#enable-verbose-logging-for-debugging)
- [SIM7600 documentation](#sim7600-documentation)
- [TODOs](#todos)
- [License](#license)

# Description

Add 4G LTE connectivity to your microcontroller projects with the **SIM7600** module. With
straightforward functions and clear examples, you'll be sending data to MQTT brokers, making HTTP requests, and
managing GPS in no time. No need to struggle with complex AT commands!

The **SIM7600** is a powerful multi-band LTE module designed for IoT and embedded applications. It supports LTE-TDD/LTE-FDD/HSPA+/GSM/GPRS/EDGE connectivity, making it ideal for data transmission, remote monitoring, asset tracking, and telemetry applications. With integrated GPS/GNSS capabilities and support for various network protocols (TCP/IP, HTTP, MQTT, FTP), the **SIM7600** serves as a comprehensive solution for projects requiring reliable cellular connectivity and positioning services. Its wide operating voltage range and low power consumption make it suitable for battery-powered applications and industrial environments.

# Usage

## Adding library to Arduino IDE

Search for the library in the Library Manager and install it.

## Adding library to platformio.ini (PlatformIO)

Search for the library in the PlatformIO Library Registry, or add the following line to your `platformio.ini` file:

```ini
; Most recent changes
lib_deps =
  https://github.com/alkonosst/SIM7600Module.git

; Release vx.y.z (using an exact version is recommended)
; Example: v1.2.3
lib_deps =
  https://github.com/alkonosst/SIM7600Module.git#vx.y.z
```

## Using the library

### Including the library

To use a feature, you need to include the corresponding header file:

```cpp
#include "SIM7600Modem.h"       // Core modem functionality
#include "SIM7600TCPClient.h"   // TCP client functionality
#include "SIM7600MQTTClient.h"  // MQTT client functionality
```

### Namespaces

All the functionality is encapsulated within the `SIM7600` namespace, so you can access the classes
and functions like this:

```cpp
SIM7600::Modem modem;
SIM7600::TCPClient tcp_client;
SIM7600::MQTTClient mqtt_client;
```

If you prefer, you can rename the namespace to something shorter:

```cpp
namespace SIM = SIM7600;

SIM::Modem modem;
SIM::TCPClient tcp_client;
SIM::MQTTClient mqtt_client;
```

Or maybe you want to use the using directive:

```cpp
using namespace SIM7600;

Modem modem;
TCPClient tcp_client;
MQTTClient mqtt_client;
```

### Status codes

The library uses the `SIM7600::Status` enum to return the status of each operation. The possible
values are inside the `SIM7600Common.h` within the `SIM7600_STATUS_LIST` macro. Almost every method
returns a `SIM7600::Status` value, so you can check if the operation was successful or if there was
an error:

```cpp
SIM7600::Status status = modem.init();

if (status == SIM7600::Status::Success) {
  // Operation was successful
} else {
  // Handle error
}
```

Also you can convert the status code to a human-readable string using the `SIM7600::statusToString`
function:

```cpp
const char* status_str = SIM7600::statusToString(status);
Serial.println(status_str);
```

### Creating an instance of the Modem and initializing it

The constructor needs a reference to a `Stream` object that represents the serial interface
connected to the SIM7600 module. If your hardware have a power control pin, first you need to enable it:

```cpp
#include "SIM7600Modem.h"

SIM7600::Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  Serial.begin(115200);

  // Initialize serial for SIM7600. Make sure to set the correct pins for your board.
  // The SIM7600 by default uses 115200 baud rate.
  Serial1.begin(115200);

  pinMode(7, OUTPUT); // Power control pin
  digitalWrite(7, HIGH); // Power on the module

  while (true) {
    Serial.println("Initializing modem...");
    SIM7600::Status status = modem.init();

    if (status == SIM7600::Status::Success) {
      Serial.println("Modem initialized successfully!");
      break;
    } else {
      Serial.print("Failed to initialize modem: ");
      Serial.println(SIM7600::statusToString(status));
    }

    delay(5000); // Retry every 5 seconds
  }
}
```

### Creating an instance of client classes

Their constructor needs a reference to a valid `SIM7600::Modem` instance:

```cpp
#include "SIM7600Modem.h"
#include "SIM7600TCPClient.h"
#include "SIM7600MQTTClient.h"

SIM7600::Modem modem(&Serial1); // Use Serial1 for communication
SIM7600::TCPClient tcp_client(&modem); // Create TCPClient instance
SIM7600::MQTTClient mqtt_client(&modem); // Create MQTTClient instance

void setup() {
  // ... Initialize modem as shown in the previous example

  // Now you can use tcp_client and mqtt_client to connect to servers and send/receive data
}
```

### Macro configuration

#### Changing TX and RX buffer sizes

Each instance of the `SIM7600::Modem` class has TX and RX buffers for communication with the SIM7600
module. You can change their sizes by defining the following macros before including the library
headers:

```cpp
// Set TX and RX buffer sizes to 512 bytes
// Default is 256 bytes for both
#define SIM7600_MODEM_TX_BUFFER_SIZE_B 512
#define SIM7600_MODEM_RX_BUFFER_SIZE_B 512
#include "SIM7600Modem.h"
```

Or if you are using PlatformIO, you can define them in the `platformio.ini` file:

```ini
build_flags =
  -DSIM7600_MODEM_TX_BUFFER_SIZE_B=512
  -DSIM7600_MODEM_RX_BUFFER_SIZE_B=512
```

#### Enabling debug output

You can enable debug output to the serial console by defining the `SIM7600_LOG_LEVEL` macro before including
the library headers. The available log levels are:

- `0` - No logging (default)
- `1` - Log only errors
- `2` - Log warnings and errors
- `3` - Log general information, warnings, and errors
- `4` - Log detailed debug information
- `5` - Log all information, including verbose debug data (AT command traffic)

```cpp
// Enable debug output at INFO level
#define SIM7600_LOG_LEVEL 3
#include "SIM7600Modem.h"
```

You can set the stream output for the logs by defining the `SIM7600_LOG_STREAM` macro before
including the library headers. By default, it uses `Serial`.

```cpp
// Set log output to Serial2
#define SIM7600_LOG_STREAM Serial2
#include "SIM7600Modem.h"
```

The default log function uses a default `256` bytes buffer for formatting log messages. You can change
its size by defining the `SIM7600_LOG_BUFFER_SIZE_B` macro before including the library headers:

```cpp
// Set log buffer size to 512 bytes
#define SIM7600_LOG_BUFFER_SIZE_B 512
#include "SIM7600Modem.h"
```

If you are using an `ESP32` platform, you can enable the native `ESP_LOGX()` logging system by defining the
`SIM7600_USE_ESP32_LOGS` macro before including the library headers:

```cpp
// Enable ESP32 native logs
#define SIM7600_USE_ESP32_LOGS
#include "SIM7600Modem.h"
```

If you need verbose error codes (`+CME ERROR:` and `+CMS ERROR:`), you can define the
`SIM7600_LOG_VERBOSE_ERROR_CODES` macro. You need to set the log level to `1` (ERROR) or higher:

```cpp
// Enable verbose error codes
#define SIM7600_LOG_VERBOSE_ERROR_CODES
#define SIM7600_LOG_LEVEL 3
#include "SIM7600Modem.h"
```

If you need a custom log function, you can define the `SIM7600_LOG_CUSTOM` to a custom function with the
following signature:

```cpp
void logFunction(char level, const char* tag, const char* format, ...);
```

```cpp
// Example of defining a custom log function
#define SIM7600_LOG_CUSTOM logFunction

#define SIM7600_LOG_LEVEL 5 // Enable all log levels
#include "SIM7600Modem.h"

void logFunction(char level, const char* tag, const char* format, ...) {
  // Your custom logging implementation here
  // See the example in SIM7600Log.cpp for reference
}

// Now you can use the library, and it will use your custom log function
```

Remember, if you are using PlatformIO, you can define these macros in the `platformio.ini` file:

```ini
build_flags =
  -DSIM7600_LOG_LEVEL=3
  -DSIM7600_LOG_BUFFER_SIZE_B=512
  -DSIM7600_USE_ESP32_LOGS
  -DSIM7600_LOG_CUSTOM=logFunction
  -DSIM7600_LOG_VERBOSE_ERROR_CODES
```

## Modem features

The `SIM7600::Modem` class provides the following features:

- Initialization and configuration of the SIM7600 module.
- Manual AT command sending and response handling.
- URC (_Unsolicited Result Code_) handling.
- Network registration and signal quality monitoring.
- GPS/GNSS management.
- NTP time synchronization.
- TCP/IP stack management.
- MQTT stack management.

> [!IMPORTANT]
> Remember to call `modem.loop()` periodically in your main loop to handle URCs (Unsolicited Result Codes) and trigger callbacks.

> [!NOTE]
> Before using TCP or MQTT services, ensure you:
>
> 1. Have initialized the modem with `modem.init()`
> 2. Are registered on the network (check with `modem.waitForNetworkRegistration()`)
> 3. Have configured the APN with `modem.configureAPN()`
> 4. Have started the corresponding service (`modem.startTCPIPService()` or `modem.startMQTTService()`)

### Initialization and status

Make sure to initialize the modem before using any of its features.

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

const char* APN = "your_apn_here";
const char* USERNAME = "your_username_here";
const char* PASSWORD = "your_password_here";

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Power on the module if necessary
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  Status status;

  while (true) {
    Serial.println("Initializing modem...");
    status = modem.init();

    if (status == Status::Success) {
      Serial.println("Modem initialized successfully!");
      break;
    } else {
      Serial.print("Failed to initialize modem: ");
      Serial.println(statusToString(status));
    }

    delay(5000); // Retry every 5 seconds
  }

  // Set APN. Username and Password are optional (nullptr by default)
  status = modem.configureAPN(APN, USERNAME, PASSWORD);
  if (status != Status::Success) {
    // Handle error
  }

  // Get signal quality
  float rssi, ber_min, ber_max;
  status = modem.getSignalQuality(rssi, ber_min, ber_max);
  if (status != Status::Success) {
    // Handle error
  }

  // Disable SMS notifications
  status = modem.disableSMSNotifications();
  if (status != Status::Success) {
    // Handle error
  }
}

void loop() {
  // IMPORTANT: Call the modem loop periodically to handle URCs
  modem.loop();
}
```

### Power management

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  // ... Initialize modem as shown in the previous example

  // Reset the modem (requires re-initialization after power on)
  Status status = modem.reset();
  if (status != Status::Success) {
    // Handle error
  }

  // Power off the modem (requires re-initialization after power on)
  status = modem.powerOff();
  if (status != Status::Success) {
    // Handle error
  }

  // If you have a SIM7600 with power control pin, turn it off as well
  digitalWrite(7, LOW);
}
```

### GPS management

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  // ... Initialize modem as shown in the previous example

  // Enable GPS auto-start
  Status status = modem.enableGPSAutoStart(true);
  if (status != Status::Success) {
    // Handle error
  }

  // If you are using an active antenna, you might want to enable the auxiliary power supply
  status = modem.setGPSAntennaVoltage(3050); // 3.05V
  if (status != Status::Success) {
    // Handle error
  }

  status = modem.enableGPSAntennaVoltage();
  if (status != Status::Success) {
    // Handle error
  }

  // Start GPS
  status = modem.enableGPS();
  if (status != Status::Success) {
    // Handle error
  }

  // Wait for GPS fix (may take 30+ seconds on cold start)
  // By default, the SIM7600 uses a 1Hz update rate, so we check constantly
  GPSData gps_data;
  uint32_t start = millis();

  while ((millis() - start) < 60000) {  // 60 seconds timeout
    Status status = modem.getGPSData(gps_data);

    if (status == Status::Success && gps_data.fix_status != GPSFixStatus::NoFix) {
      Serial.printf("GPS fixed! Lat/Lon: %.6lf, %.6lf\n", gps_data.latitude, gps_data.longitude);
      break;
    }

    delay(2000);  // Check every 2 seconds
  }
}
```

### Network management

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  // ... Initialize modem as shown in the previous example

  // Wait until registered on the network. This may take some time.
  RegStatus reg_status;
  Status status = modem.waitForNetworkRegistration(reg_status, 60000); // 60 seconds timeout
  if (status != Status::Success) {
    // Handle error
  }

  // Check if registered
  bool registered;
  status = modem.isRegisteredOnNetwork(registered);
  if (status != Status::Success) {
    // Handle error
  }

  if (registered) {
    Serial.println("Modem is registered on the network.");
  } else {
    Serial.println("Modem is not registered on the network.");
  }

  // Get last known registration status without querying the modem
  Serial.printf("Currently registered on network: %s, Status: %u\r\n",
        modem.isCurrentlyRegisteredOnNetwork() ? "Yes" : "No",
        static_cast<uint8_t>(modem.getCurrentRegistrationStatus()));
}
```

### NTP time synchronization

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  // ... Initialize modem as shown in the previous example

  // Set NTP server, UTC time zone = 0
  Status status = modem.setNTPServer("pool.ntp.org", 0);
  if (status != Status::Success) {
    // Handle error
  }

  // Synchronize time
  NTPSyncStatus ntp_status;
  status = modem.synchronizeTime(ntp_status);
  if (status != Status::Success) {
    // Handle error
  }

  // Get current time
  NTPTimeData time_data;
  status = modem.getNetworkTime(time_data);
  if (status == Status::Success) {
    Serial.printf("Current time: %04u-%02u-%02u %02u:%02u:%02u\n",
                  time_data.year, time_data.month, time_data.day,
                  time_data.hour, time_data.minute, time_data.second);
  } else {
    // Handle error
  }
}
```

### Start/stop services

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

void setup() {
  // ... Initialize modem as shown in the previous example

  // Start TCP/IP service
  Status status = modem.startTCPIPService();
  if (status != Status::Success) {
    // Handle error
  }

  // Stop TCP/IP service
  status = modem.stopTCPIPService();
  if (status != Status::Success) {
    // Handle error
  }

  // Start MQTT service
  status = modem.startMQTTService();
  if (status != Status::Success) {
    // Handle error
  }

  // Stop MQTT service
  status = modem.stopMQTTService();
  if (status != Status::Success) {
    // Handle error
  }
}
```

### Callbacks

The `SIM7600::Modem` class allows you to set callbacks for various services events, such as TCP/IP
and MQTT. You need to define your callback functions with the appropriate signatures and then set them using the
`set...Callback` methods.

> [!IMPORTANT]
> Whenever a service closes (TCP/IP or MQTT), you need to start it again before using it.

```cpp
#include "SIM7600Modem.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication

// Modem ready event callback
void modemReadyCB() { Serial.println("Event: Modem is ready!"); }

// Network changed event callback
void networkChangedCB(const bool registered, const RegStatus reg_status) {
  Serial.printf("Event: Network changed! Registered: %s, Status: %u\r\n",
    registered ? "Yes" : "No",
    static_cast<uint8_t>(reg_status));
}

// TCP/IP closed event callback
// IMPORTANT: If this event occurs, you need to start the TCP/IP service again before using it.
void tcpNetworkClosedCB() { Serial.println("Event: TCP network connection closed!"); }

// MQTT closed event callback
// IMPORTANT: If this event occurs, you need to start the MQTT service again before using it.
void mqttNetworkClosedCB() { Serial.println("Event: MQTT network connection closed!"); }

void setup() {
  // ... Initialize modem as shown in the previous example

  // Set modem ready event callback
  modem.setModemReadyCallback(modemReadyCB);

  // Set network changed event callback
  modem.setNetworkChangedCallback(networkChangedCB);

  // Set TCP/IP closed event callback
  modem.setTCPNetworkClosedCallback(tcpNetworkClosedCB);

  // Set MQTT closed event callback
  modem.setMQTTNetworkClosedCallback(mqttNetworkClosedCB);
}
```

## TCPClient features

The `SIM7600::TCPClient` class provides the following features:

- Derives from Arduino `Client` class, allowing integration with libraries that use it.
- Have up to 10 simultaneous TCP connections.
- Connect to TCP servers using domain names or IP addresses.
- Send and receive data over TCP connections.

> [!IMPORTANT]
> Don't forget to call `modem.loop()` in your main loop to process URCs and trigger callbacks:
>
> ```cpp
> void loop() {
>   modem.loop();
>   // Your code here
> }
> ```

### Connecting to a TCP server

```cpp
#include "SIM7600Modem.h"
#include "SIM7600TCPClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
TCPClient tcp_client(&modem); // Create TCPClient instance

void setup() {
  // ... Initialize modem as shown in the previous example
  // Remember to start the TCP/IP service before connecting

  const char* server = "www.example.com";
  uint16_t port = 80;

  Serial.printf("Connecting to %s:%u...\n", server, port);

  // Connect to the server
  Status status = tcp_client.connect(server, port);
  if (status != Status::Success) {
    // Handle error
  }

  // Check if connected
  bool connected;
  status = tcp_client.isConnected(connected);
  if (status != Status::Success) {
    // Handle error
  }

  // Disconnect from the server
  status = tcp_client.disconnect();
  if (status != Status::Success) {
    // Handle error
  }
}
```

### Transferring data

```cpp
#include "SIM7600Modem.h"
#include "SIM7600TCPClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
TCPClient tcp_client(&modem); // Create TCPClient instance

void setup() {
  // ... Initialize modem and connect to server as shown in the previous example
  // Remember to start the TCP/IP service before connecting

  const char* http_request = "GET / HTTP/1.1\r\nHost: www.example.com\r\nConnection: close\r\n\r\n";

  // Send HTTP request
  size_t bytes_sent;
  Status status = tcp_client.send((const uint8_t*)http_request, strlen(http_request), bytes_sent);
  if (status != Status::Success) {
    // Handle error
  }

  Serial.printf("Sent %u bytes\n", bytes_sent);

  // Checking available data to read
  size_t available;
  status = tcp_client.getAvailableBytes(available);
  if (status != Status::Success) {
    // Handle error
  }

  // Receive HTTP response (leave 1 byte for null-terminator)
  uint8_t buffer[512];
  size_t bytes_received;
  status = tcp_client.readData(buffer, sizeof(buffer) - 1, bytes_received);
  if (status != Status::Success) {
    // Handle error
  }

  buffer[bytes_received] = '\0'; // Null-terminate the received data
  Serial.printf("Received %u bytes:\n%s\n", bytes_received, buffer);

  // If you send the header "Connection: close", the server will close the connection after the response.
  // So you can check if the connection is still open:
  bool connected;
  status = tcp_client.isConnected(connected);
  if (status != Status::Success) {
    // Handle error
  }

  if (!connected) {
    Serial.println("Connection closed by server.");
  }
}
```

### Callbacks

The `SIM7600::TCPClient` class allows you to set callbacks for data reception and connection closed events:

```cpp
#include "SIM7600Modem.h"
#include "SIM7600TCPClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
TCPClient tcp_client(&modem); // Create TCPClient instance

// Data received event callback
void tcpDataReceivedCB(TCPClient* const client) {
  Serial.printf("Event: Data received on TCP client %u\n", client->getClientID());
}

// Connection closed event callback
// The reason parameter indicates why the connection was closed
// (By the remote server, locally or timeout)
void tcpConnectionClosedCB(TCPClient* const client, const TCPCloseReason reason) {
  Serial.printf("Event: TCP client %u connection closed! Reason: %u\n", client->getClientID(), static_cast<uint8_t>(reason));
}

void setup() {
  // ... Initialize modem as shown in the previous example
  // Remember to start the TCP/IP service before connecting

  // Set data received event callback
  tcp_client.setDataReceivedCallback(tcpDataReceivedCB);

  // Set connection closed event callback
  tcp_client.setConnectionClosedCallback(tcpConnectionClosedCB);
}
```

## MQTTClient features

The `SIM7600::MQTTClient` class provides the following features:

- Have up to 2 simultaneous MQTT connections.
- Connect to MQTT brokers using domain names or IP addresses.
- Select between MQTT v3.1 and MQTT v3.1.1 protocols.
- Choose a QoS level for message delivery (0, 1, or 2).
- Set a will message.
- Publish and subscribe to topics.

> [!IMPORTANT]
> Don't forget to call `modem.loop()` in your main loop to process URCs and trigger callbacks:
>
> ```cpp
> void loop() {
>   modem.loop();
>   // Your code here
> }
> ```

### Connecting to an MQTT broker

```cpp
#include "SIM7600Modem.h"
#include "SIM7600MQTTClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
MQTTClient mqtt_client(&modem); // Create MQTTClient instance

void setup() {
  // ... Initialize modem as shown in the previous example
  // Remember to start the MQTT service before connecting

  // Acquire a client
  const char* client_id = "my_mqtt_client";
  bool use_tls = false;
  MQTTVersion version = MQTTVersion::V3_1_1;

  Status status = mqtt_client.acquireClient(client_id, use_tls, version);
  if (status != Status::Success) {
    // Handle error
  }

  // Set the will message (optional)
  const char* will_topic = "test/will";
  const char* will_message = "Client disconnected unexpectedly";
  // MQTTQoS qos = MQTTQoS::AtMostOnce;   // QoS 0: Fire and forget
  MQTTQoS qos = MQTTQoS::AtLeastOnce;     // QoS 1: Acknowledged delivery
  // MQTTQoS qos = MQTTQoS::ExactlyOnce;  // QoS 2: Assured delivery

  status = mqtt_client.setLastWillMessage(will_topic, will_message, will_qos);
  if (status != Status::Success) {
    // Handle error
  }

  // Connect to the MQTT broker
  const char* broker = "test.mosquitto.org";
  uint16_t port = 1883;

  Serial.printf("Connecting to MQTT broker %s:%u...\n", broker, port);

  Status status = mqtt_client.connect(broker, port);
  if (status != Status::Success) {
    // Handle error
  }

  // Check connection
  bool connected;
  status = mqtt_client.isConnected(connected);
  if (status != Status::Success) {
    // Handle error
  }

  // Disconnect from the broker
  status = mqtt_client.disconnect();
  if (status != Status::Success) {
    // Handle error
  }
}
```

### Publishing and subscribing to topics

```cpp
#include "SIM7600Modem.h"
#include "SIM7600MQTTClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
MQTTClient mqtt_client(&modem); // Create MQTTClient instance

void setup() {
  // ... Initialize modem and connect to MQTT broker as shown in the previous example
  // Remember to start the MQTT service before connecting

  const char* topic = "test/topic";
  const char* message = "Hello, MQTT!";
  // MQTTQoS qos = MQTTQoS::AtMostOnce;   // QoS 0: Fire and forget
  MQTTQoS qos = MQTTQoS::AtLeastOnce;     // QoS 1: Acknowledged delivery
  // MQTTQoS qos = MQTTQoS::ExactlyOnce;  // QoS 2: Assured delivery

  // Publish a message
  Status status = mqtt_client.publish(topic, (const uint8_t*)message, strlen(message), qos);
  if (status != Status::Success) {
    // Handle error
  }

  // Subscribe to a topic
  status = mqtt_client.subscribe(topic, qos);
  if (status != Status::Success) {
    // Handle error
  }
}
```

### Callbacks

The `SIM7600::MQTTClient` class allows you to set callbacks for message reception and connection
closed events. Unlike the `TCPClient`, it is not optional to set these callbacks, as they are the
only way to receive messages of subscribed topics:

```cpp
#include "SIM7600Modem.h"
#include "SIM7600MQTTClient.h"
using namespace SIM7600;
Modem modem(&Serial1); // Use Serial1 for communication
MQTTClient mqtt_client(&modem); // Create MQTTClient instance

// Connection lost callback
void mqttConnectionLostCB(MQTTClient* const client, const MQTTConnLostCause cause) {
  Serial.printf("Event: MQTT client %u connection lost! Reason: %u\n",
                client->getClientID(), static_cast<uint8_t>(cause));
}

// Ping failed callback
void mqttPingFailedCB(MQTTClient* const client) {
  Serial.printf("Event: MQTT client %u ping failed!\n",
                client->getClientID());
}

// Receive started callback
void mqttRxStartCB(MQTTClient* const client, const size_t topic_len, const size_t payload_len) {
  Serial.printf("Event: MQTT client %u started receiving a message. Topic length: %u, Payload length: %u\n",
                client->getClientID(), topic_len, payload_len);
}

// Topic message received callback
void mqttRxTopicCB(MQTTClient* const client, const char* topic, const size_t topic_len) {
  Serial.printf("Event: MQTT client %u received message on topic: %s (%u)\n",
                client->getClientID(), topic, topic_len);
}

// Payload message received callback
void mqttRxPayloadCB(MQTTClient* const client, const uint8_t* payload, const size_t payload_len) {
  Serial.printf("Event: MQTT client %u received payload: %s (%u)\n",
                client->getClientID(), payload, payload_len);
}

// Message received complete callback
void mqttRxEndCB(MQTTClient* const client) {
  Serial.printf("Event: MQTT client %u finished receiving a message.\n",
                client->getClientID());
}

void setup() {
  // ... Initialize modem as shown in the previous example
  // Remember to start the MQTT service before connecting

  // Set connection lost event callback
  mqtt_client.setConnectionLostCallback(mqttConnectionLostCB);

  // Set ping failed event callback
  mqtt_client.setPingFailedCallback(mqttPingFailedCB);

  // Set receive started event callback
  mqtt_client.setRxStartCallback(mqttRxStartCB);

  // Set topic message received event callback
  mqtt_client.setRxTopicCallback(mqttRxTopicCB);

  // Set payload message received event callback
  mqtt_client.setRxPayloadCallback(mqttRxPayloadCB);

  // Set message received complete event callback
  mqtt_client.setRxEndCallback(mqttRxEndCB);
}
```

# Troubleshooting

## Module doesn't respond

- Check power supply (4V typical, up to 2A peak current during transmission).
- Verify TX/RX are connected correctly (module TX → MCU RX, module RX → MCU TX).
- Ensure baud rate is 115200 (default for SIM7600).
- Try power cycling the module.

## Can't register on network

- Check SIM card is properly inserted and unlocked.
- Verify APN configuration is correct for your carrier.
- Check antenna connection.
- Use `getSignalQuality()` to verify signal strength (RSSI > -100 dBm recommended).

## GPS doesn't get a fix

- Ensure GPS antenna has clear view of the sky.
- GPS cold start can take 30+ seconds.
- If using active antenna, ensure it's powered correctly.
- Use `getGPSData()` repeatedly until `fix_status` indicates a valid fix.

## MQTT/TCP connection fails

- Verify you're registered on the network first.
- Check APN is configured correctly.
- Ensure the service is started (`startTCPIPService()` or `startMQTTService()`).
- Verify broker/server address and port are correct.
- Check firewall rules if using custom server.
- Check MQTT credentials.

## Enable verbose logging for debugging

```cpp
#define SIM7600_LOG_LEVEL 5  // Enable all logs including AT commands
#include "SIM7600Modem.h"
```

# SIM7600 documentation

For more information about the **SIM7600** module, please refer to the [official
documentation](https://www.simcom.com/product/SIM7600X-H.html) on the **SIMCom website**.

# TODOs

Some features are not yet implemented. Planned features include:

- [ ] Implement reading modem information (IMEI, IMSI, provider, etc.).
- [ ] Implement SMS management.
- [ ] Implement SSL service.
- [ ] Implement HTTP service.
- [ ] Implement MQTT over SSL.

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
