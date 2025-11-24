/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "SIM7600Modem.h"
#include "SIM7600Log.h"

static const char* tag = "SIM7600Modem";

namespace SIM7600 {

Modem::Modem()
    : _serial(nullptr)
    , _cb_modem_ready(nullptr)
    , _cb_network_changed(nullptr)
    , _cb_tcp_network_closed(nullptr)
    , _cb_mqtt_network_closed(nullptr)
    , _registered_on_network(false)
    , _current_reg_status(RegStatus::NotRegisteredAndNotSearching)
    , _urc_handler_count(0) {
  memset(_tx_buf, 0, SIM7600_MODEM_TX_BUFFER_SIZE_B);
  memset(_rx_buf, 0, SIM7600_MODEM_RX_BUFFER_SIZE_B);
  memset(_urc_handlers, 0, sizeof(_urc_handlers));
}

Modem::Modem(Stream* serial)
    : _serial(serial)
    , _cb_modem_ready(nullptr)
    , _cb_network_changed(nullptr)
    , _cb_tcp_network_closed(nullptr)
    , _cb_mqtt_network_closed(nullptr)
    , _registered_on_network(false)
    , _current_reg_status(RegStatus::NotRegisteredAndNotSearching)
    , _urc_handler_count(0) {
  memset(_tx_buf, 0, SIM7600_MODEM_TX_BUFFER_SIZE_B);
  memset(_rx_buf, 0, SIM7600_MODEM_RX_BUFFER_SIZE_B);
  memset(_urc_handlers, 0, sizeof(_urc_handlers));
}

void Modem::setSerialPort(Stream* serial) { _serial = serial; }

Status Modem::setModemReadyCallback(ModemReadyCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_modem_ready = callback;
  return Status::Success;
}

Status Modem::setNetworkChangedCallback(NetworkChangedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_network_changed = callback;
  return Status::Success;
}

Status Modem::setTCPNetworkClosedCallback(TCPNetworkClosedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_tcp_network_closed = callback;
  return Status::Success;
}

Status Modem::setMQTTNetworkClosedCallback(MQTTNetworkClosedCB callback) {
  if (callback == nullptr) return Status::InvalidCallback;
  _cb_mqtt_network_closed = callback;
  return Status::Success;
}

Status Modem::init(const char* pin, const uint32_t timeout_ms) {
  SIM7600_LOGI(tag, "Initializing SIM7600 modem");

  // Test AT communication
  Status status = testAT(timeout_ms);
  if (status != Status::Success) return status;

  // Disable echo
  SIM7600_LOGD(tag, "Disabling echo");
  status = sendATCmdAndWaitResp("ATE0", AT_OK);
  if (status != Status::Success) return status;

#if SIM7600_LOG_VERBOSE_ERROR_CODES
  // Enable verbose errors
  SIM7600_LOGD(tag, "Enabling verbose errors");
  status = sendATCmdAndWaitResp("AT+CMEE=2", AT_OK);
  if (status != Status::Success) return status;
#else
  // Disable verbose errors
  SIM7600_LOGD(tag, "Disabling verbose errors");
  status = sendATCmdAndWaitResp("AT+CMEE=0", AT_OK);
  if (status != Status::Success) return status;
#endif

  // Enable CGREG URCs
  SIM7600_LOGD(tag, "Enabling CGREG URCs");
  status = sendATCmdAndWaitResp("AT+CGREG=1", AT_OK);
  if (status != Status::Success) return status;

  // TCPIP receive mode set to manual
  SIM7600_LOGD(tag, "Setting TCPIP receive mode to manual");
  status = sendATCmdAndWaitResp("AT+CIPRXGET=1", AT_OK);
  if (status != Status::Success) return status;

  // Configure socket parameters
  // +CIPCCFG: <NmRetry>,<DelayTm>,<Ack>,<errMode>,<HeaderType>,<AsyncMode>,<TimeoutVal>
  SIM7600_LOGD(tag, "Configuring socket parameters");
  status = sendATCmdAndWaitResp("AT+CIPCCFG=10,0,0,1,1,0,120000", AT_OK);
  if (status != Status::Success) return status;

  // Check if PIN is required
  SIMStatus sim_status;
  status = getSIMStatus(sim_status);
  if (status != Status::Success) return status;

  // No PIN required
  if (sim_status == SIMStatus::Ready) return Status::Success;

  // No PIN required but probably PUK is needed or SIM error
  if (sim_status == SIMStatus::Error) return Status::Error;

  // PIN is required but not provided
  if (pin == nullptr) {
    SIM7600_LOGE(tag, "SIM PIN required but not provided");
    return Status::InvalidPIN;
  }

  // Unlock SIM with provided PIN
  status = unlockSIM(pin);
  if (status != Status::Success) return status;

  // Verify SIM status again
  status = getSIMStatus(sim_status);
  if (status != Status::Success) return status;

  if (sim_status != SIMStatus::Ready) {
    SIM7600_LOGE(tag, "PIN required to unlock SIM");
    return Status::PINRequired;
  }

  SIM7600_LOGI(tag, "Modem initialized successfully");
  return Status::Success;
}

Status Modem::powerOff() {
  SIM7600_LOGI(tag, "Powering off SIM7600 modem");

  // Send power off command
  return sendATCmdAndWaitResp("AT+CPOF", AT_OK);
}

Status Modem::reset() {
  SIM7600_LOGI(tag, "Resetting SIM7600 modem");

  // Send reset command
  return sendATCmdAndWaitResp("AT+CRESET", AT_OK);
}

Status Modem::sendATCmd(const char* cmd_format, ...) {
  va_list args;
  va_start(args, cmd_format);
  int16_t len = vsnprintf(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B, cmd_format, args);
  va_end(args);

  // Leave space for CR+LF
  if (len <= 0 || (len >= SIM7600_MODEM_TX_BUFFER_SIZE_B - 2)) {
    SIM7600_LOGE(tag, "AT command buffer overflow");
    return Status::BufferOverflow;
  }

  // Append CR+LF
  strcat(_tx_buf, AT_NL);

  return _writeATCmd();
}

Status Modem::sendATCmdAndWaitResp(const char* cmd, const char* expected_response,
  const uint32_t timeout_ms) {
  // Write AT command
  Status status = sendATCmd(cmd);
  if (status != Status::Success) return status;

  return waitForResponse(expected_response, timeout_ms);
}

Status Modem::sendATCmdAndWaitResp(const char* cmd, const char** expected_responses,
  const uint8_t response_count, uint8_t& found_index, const uint32_t timeout_ms) {
  // Write AT command
  Status status = sendATCmd(cmd);
  if (status != Status::Success) return status;

  return waitForResponses(expected_responses, response_count, found_index, timeout_ms);
}

Status Modem::waitForResponse(const char* expected_response, const uint32_t timeout_ms) {
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    uint32_t elapsed = millis() - start;
    if (elapsed >= timeout_ms) break;

    uint32_t remaining_time_ms = timeout_ms - elapsed;

    Status status = readLine(_rx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, remaining_time_ms);

    if (status == Status::EmptyLine) continue;
    if (status != Status::Success) return status;

    // Check for expected response
    if (strstr(_rx_buf, expected_response) != nullptr) {
      return Status::Success;
    }

    // Check for error response
    if (_receivedErrorResponse()) return Status::Error;
  }

  SIM7600_LOGE(tag, "Timed out expecting response");
  return Status::Timeout;
}

Status Modem::waitForResponses(const char** expected_responses, const uint8_t response_count,
  uint8_t& found_index, const uint32_t timeout_ms) {
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    uint32_t elapsed = millis() - start;
    if (elapsed >= timeout_ms) break;

    uint32_t remaining_time_ms = timeout_ms - elapsed;

    Status status = readLine(_rx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, remaining_time_ms);

    if (status == Status::EmptyLine) continue;
    if (status != Status::Success) return status;

    // Check for expected responses
    for (uint8_t i = 0; i < response_count; i++) {
      if (strstr(_rx_buf, expected_responses[i]) != nullptr) {
        found_index = i;
        return Status::Success;
      }
    }

    // Check for error response
    if (_receivedErrorResponse()) return Status::Error;
  }

  SIM7600_LOGE(tag, "Timed out expecting %u responses", response_count);
  return Status::Timeout;
}

Status Modem::waitForPrompt(const uint32_t timeout_ms) {
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    uint32_t elapsed = millis() - start;
    if (elapsed >= timeout_ms) break;

    uint32_t remaining_time_ms = timeout_ms - elapsed;

    size_t bytes_read = 0;
    Status status     = readBytes(reinterpret_cast<uint8_t*>(_rx_buf),
      SIM7600_MODEM_RX_BUFFER_SIZE_B,
      1,
      bytes_read,
      remaining_time_ms);

    if (status != Status::Success) return status;

    // Check for prompt character
    if (_rx_buf[0] == '>') {
      _rx_buf[1] = '\0';
      SIM7600_LOGV(tag, "<< %s", _rx_buf);
      return Status::Success;
    }
  }

  SIM7600_LOGE(tag, "Timed out waiting for prompt");
  return Status::Timeout;
}

Status Modem::readLine(char* const buffer, const size_t buffer_size, const uint32_t timeout_ms) {
  if (_serial == nullptr) return Status::NoSerialPort;
  if (buffer == nullptr || buffer_size == 0) return Status::InvalidBuffer;

  size_t index   = 0;
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    // Yield to other tasks
    if (_serial->available() == 0) {
      yield();
      continue;
    }

    char c = _serial->read();

    // Ignore carriage return except line feed
    if (c == '\r') continue;

    // End of line
    if (c == '\n') {
      buffer[index] = '\0';
      SIM7600_LOGV(tag, "<< %s", buffer);

      // Empty line
      if (index == 0) {
        return Status::EmptyLine;
      }

      // Process URCs here if captured
      bool urc_captured = _handleURCs();

      // Reset buffer for next line if URC captured
      if (urc_captured) {
        index = 0;
        continue;
      }

      return Status::Success;
    }

    // Store character in buffer if space is available
    if (index < buffer_size - 1) {
      buffer[index++] = c;
      continue;
    }

    SIM7600_LOGE(tag, "AT command buffer overflow");
    return Status::BufferOverflow;
  }

  return Status::Timeout;
}

Status Modem::readBytes(uint8_t* const buffer, const size_t buffer_size, const size_t bytes_to_read,
  size_t& bytes_read, const uint32_t timeout_ms) {
  if (_serial == nullptr) return Status::NoSerialPort;
  if (buffer == nullptr || buffer_size == 0) return Status::InvalidBuffer;

  bytes_read     = 0;
  size_t index   = 0;
  uint32_t start = millis();

  while ((millis() - start) < timeout_ms) {
    // Check if all requested bytes have been read
    if (index >= bytes_to_read) {
      return Status::Success;
    }

    // Yield to other tasks
    if (_serial->available() == 0) {
      yield();
      continue;
    }

    // Check for overflow
    if (index >= buffer_size) {
      SIM7600_LOGE(tag, "AT command buffer overflow");
      return Status::BufferOverflow;
    }

    // Read byte
    uint8_t byte_read = _serial->read();

    buffer[index++] = static_cast<uint8_t>(byte_read);
    bytes_read      = index;
  }

  return Status::Timeout;
}

Status Modem::parseLine(char* const buffer, const uint8_t parameters, const char* format, ...) {
  if (buffer == nullptr || format == nullptr) return Status::InvalidBuffer;
  if (parameters == 0) return Status::InvalidParameter;

  va_list args;
  va_start(args, format);
  int parsed = vsscanf(buffer, format, args);
  va_end(args);

  if (parsed < parameters) {
    return Status::InvalidResponse;
  }

  return Status::Success;
}

Status Modem::testAT(const uint32_t timeout_ms) {
  SIM7600_LOGD(tag, "Testing AT communication");

  Status status;

  for (uint32_t start = millis(); (millis() - start) < timeout_ms;) {
    status = sendATCmdAndWaitResp("AT", AT_OK);

    if (status == Status::Success) {
      SIM7600_LOGD(tag, "AT communication successful");
      return Status::Success;
    }
  }

  SIM7600_LOGE(tag, "AT communication test timed out");
  return Status::Timeout;
}

Status Modem::getSIMStatus(SIMStatus& sim_status) {
  SIM7600_LOGD(tag, "Checking SIM status");

  const char* responses[] = {"+CPIN: READY", "+CPIN: SIM PIN"};

  uint8_t found_index = 0;
  Status status       = sendATCmd("AT+CPIN?", responses, 2, found_index);
  if (status != Status::Success) return status;

  switch (found_index) {
    case 0:
      sim_status = SIMStatus::Ready;
      SIM7600_LOGD(tag, "SIM status: Ready");
      break;
    case 1:
      sim_status = SIMStatus::PinWaiting;
      SIM7600_LOGD(tag, "SIM status: PIN waiting");
      break;
    default:
      sim_status = SIMStatus::Error;
      SIM7600_LOGE(tag, "SIM status: Error");
      break;
  }

  return waitForResponse(AT_OK);
}

Status Modem::unlockSIM(const char* pin) {
  SIM7600_LOGD(tag, "Unlocking SIM with provided PIN");

  Status status = sendATCmd("AT+CPIN=\"%s\"", pin);
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::getSignalQuality(float& rssi, float& ber_min, float& ber_max) {
  SIM7600_LOGD(tag, "Getting signal quality");

  rssi    = 0;
  ber_min = 0;
  ber_max = 0;

  Status status = sendATCmdAndWaitResp("AT+CSQ", "+CSQ:");
  if (status != Status::Success) return status;

  // Expected response: +CSQ: <rssi>,<ber>
  uint8_t rssi_code, ber_code;
  status = parseLine(_rx_buf, 2, "+CSQ: %hhu,%hhu", &rssi_code, &ber_code);
  if (status != Status::Success) return status;

  // Convert RSSI code to dBm
  // Supported codes: 0-31 (-113 to -51dBm), 99 (not known or not detectable)
  if ((rssi_code == 0) || (rssi_code <= 31)) {
    rssi = -113.0f + 2.0f * rssi_code;
  } else {
    rssi = 0.0f;
  }

  // Convert BER code to percentage
  // Supported codes: 0-7 (<0.01 to >=8.0%), 99 (not known or not detectable)
  switch (ber_code) {
    case 0:
      ber_min = 0.0f;
      ber_max = 0.01f;
      break;
    case 1:
      ber_min = 0.01f;
      ber_max = 0.1f;
      break;
    case 2:
      ber_min = 0.1f;
      ber_max = 0.5f;
      break;
    case 3:
      ber_min = 0.5f;
      ber_max = 1.0f;
      break;
    case 4:
      ber_min = 1.0f;
      ber_max = 2.0f;
      break;
    case 5:
      ber_min = 2.0f;
      ber_max = 4.0f;
      break;
    case 6:
      ber_min = 4.0f;
      ber_max = 8.0f;
      break;
    case 7:
      ber_min = 8.0f;
      ber_max = 100.0f;
      break;
    default: // Not known or not detectable
      ber_min = 0.0f;
      ber_max = 0.0f;
      break;
  }

  return waitForResponse(AT_OK);
}

Status Modem::disableSMSNotifications() {
  SIM7600_LOGD(tag, "Disabling SMS notifications");
  return sendATCmdAndWaitResp("AT+CNMI=0,0,0,0,0", AT_OK);
}

Status Modem::setGPSAntennaVoltage(const uint16_t voltage_mv) {
  SIM7600_LOGD(tag, "Setting GPS antenna voltage to: %u mV", voltage_mv);

  if (voltage_mv < SIM7600_MODEM_MIN_GPS_ANTENNA_VOLTAGE_MV ||
      voltage_mv > SIM7600_MODEM_MAX_GPS_ANTENNA_VOLTAGE_MV) {
    SIM7600_LOGE(tag, "Invalid GPS antenna voltage: %u mV", voltage_mv);
    return Status::InvalidAntennaVoltage;
  }

  Status status = sendATCmd("AT+CVAUXV=%u", voltage_mv);
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::getGPSAntennaVoltage(uint16_t& voltage_mv) {
  SIM7600_LOGD(tag, "Getting GPS antenna voltage");

  voltage_mv = 0;

  Status status = sendATCmdAndWaitResp("AT+CVAUXV?", "+CVAUXV:");
  if (status != Status::Success) return status;

  status = parseLine(_rx_buf, 1, "+CVAUXV: %hu", &voltage_mv);
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::enableGPSAntennaVoltage() {
  SIM7600_LOGD(tag, "Enabling GPS antenna voltage");
  return sendATCmdAndWaitResp("AT+CVAUXS=1", AT_OK);
}

Status Modem::disableGPSAntennaVoltage() {
  SIM7600_LOGD(tag, "Disabling GPS antenna voltage");
  return sendATCmdAndWaitResp("AT+CVAUXS=0", AT_OK);
}

Status Modem::isGPSAntennaVoltageEnabled(bool& enabled) {
  SIM7600_LOGD(tag, "Checking if GPS antenna voltage is enabled");

  enabled = false;

  Status status = sendATCmdAndWaitResp("AT+CVAUXS?", "+CVAUXS:");
  if (status != Status::Success) return status;

  uint8_t auxv_status = 0;
  status              = parseLine(_rx_buf, 1, "+CVAUXS: %hhu", &auxv_status);
  if (status != Status::Success) return status;

  enabled = (auxv_status == 1);

  return waitForResponse(AT_OK);
}

Status Modem::enableGPS() {
  SIM7600_LOGD(tag, "Enabling GPS");

  // Check if GPS is enabled
  bool enabled;
  Status status = isGPSEnabled(enabled);
  if (status != Status::Success) return status;

  if (enabled) {
    SIM7600_LOGW(tag, "GPS is already enabled");
    return Status::Success;
  }

  return sendATCmdAndWaitResp("AT+CGPS=1,1", AT_OK);
}

Status Modem::disableGPS() {
  SIM7600_LOGD(tag, "Disabling GPS");

  // Check if GPS is enabled
  bool enabled;
  Status status = isGPSEnabled(enabled);
  if (status != Status::Success) return status;

  // Disable GPS
  status = sendATCmd("AT+CGPS=0");
  if (status != Status::Success) return status;

  // When already disabled, it only responds with "OK"
  if (!enabled) {
    return waitForResponse(AT_OK);
  }

  // When enabled, it responds with "OK" and "+CGPS: 0"
  return waitForResponse("+CGPS: 0", 10000);
}

Status Modem::isGPSEnabled(bool& enabled) {
  SIM7600_LOGD(tag, "Checking if GPS is enabled");

  enabled = false;

  Status status = sendATCmdAndWaitResp("AT+CGPS?", "+CGPS:");
  if (status != Status::Success) return status;

  uint8_t gps_status = 0;
  status             = parseLine(_rx_buf, 1, "+CGPS: %hhu", &gps_status);
  if (status != Status::Success) return status;

  enabled = (gps_status == 1);

  return waitForResponse(AT_OK);
}

Status Modem::enableGPSAutoStart(const bool enable) {
  SIM7600_LOGD(tag, "Setting GPS auto start to: %s", enable ? "ENABLED" : "DISABLED");

  Status status = sendATCmd("AT+CGPSAUTO=%u", enable ? 1 : 0);
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::getGPSAutoStart(bool& enabled) {
  SIM7600_LOGD(tag, "Getting GPS auto start status");

  enabled = false;

  Status status = sendATCmdAndWaitResp("AT+CGPSAUTO?", "+CGPSAUTO:");
  if (status != Status::Success) return status;

  uint8_t auto_start = 0;
  status             = parseLine(_rx_buf, 1, "+CGPSAUTO: %hhu", &auto_start);
  if (status != Status::Success) return status;

  enabled = (auto_start == 1);

  return waitForResponse(AT_OK);
}

Status Modem::getGPSData(GPSData& gps_data) {
  SIM7600_LOGD(tag, "Getting GPS data");

  // Set default values
  gps_data = GPSData();

  Status status = sendATCmdAndWaitResp("AT+CGNSSINFO", "+CGNSSINFO:");
  if (status != Status::Success) return status;

  // Format: +CGNSSINFO:[<mode>],[<GPS-SVs>],[<GLONASS-SVs>],[<BEIDOU-SVs>],
  // [<lat>],[<N/S>],[<log>],[<E/W>],[<date>],[<UTC-time>],[<alt>],
  // [<speed>],[<course>],[<PDOP>],[HDOP],[VDOP]

  // Example: +CGNSSINFO: 2,09,05,00,3113.330650,N,12121.262554,E,131117,091918.0,32.9,
  // 0.0,255.0,1.1,0.8,0.7

  // Not fixed yet check
  if (strstr(_rx_buf, "+CGNSSINFO: ,") != nullptr) {
    SIM7600_LOGW(tag, "GPS data not fixed yet");

    // Wait for OK
    status = waitForResponse(AT_OK);
    if (status != Status::Success) return status;

    return Status::GPSNotFixed;
  }

  char* token = _rx_buf + strlen("+CGNSSINFO:");
  char* comma;
  uint8_t index = 0;

  char n_s = 'N';
  char e_w = 'E';

  while ((comma = strchr(token, ','))) {
    *comma = '\0';

    switch (index) {
      case 0:
      {
        GPSFixStatus fix_mode;

        if (sscanf(token, "%u", &fix_mode) != 1) {
          SIM7600_LOGE(tag, "Failed to parse GPS fix mode");

          // Wait for OK
          status = waitForResponse(AT_OK);
          if (status != Status::Success) return status;

          return Status::InvalidResponse;
        }

        gps_data.fix_status = fix_mode;
      } break;

      case 1: sscanf(token, "%u", &gps_data.gps_satellites); break;
      case 2: sscanf(token, "%u", &gps_data.glonass_satellites); break;
      case 3: sscanf(token, "%u", &gps_data.beidou_satellites); break;
      case 4: sscanf(token, "%lf", &gps_data.latitude); break;
      case 5: n_s = token[0]; break;
      case 6: sscanf(token, "%lf", &gps_data.longitude); break;
      case 7: e_w = token[0]; break;
      case 8:
        sscanf(token, "%2hhu%2hhu%2hhu", &gps_data.day, &gps_data.month, &gps_data.year);
        gps_data.year += 2000;
        break;
      case 9:
      {
        float sec;
        sscanf(token, "%2hhu%2hhu%f", &gps_data.hour, &gps_data.minute, &sec);
        gps_data.second = static_cast<uint8_t>(sec);
        break;
      }
      case 10: sscanf(token, "%f", &gps_data.altitude); break;
      case 11: sscanf(token, "%f", &gps_data.speed); break;
      case 12: sscanf(token, "%f", &gps_data.course); break;
      case 13: sscanf(token, "%f", &gps_data.pdop); break;
      case 14: sscanf(token, "%f", &gps_data.hdop); break;
    }

    token = comma + 1;
    index++;
  }

  // Last value (VDOP)
  sscanf(token, "%f", &gps_data.vdop);

  // Check fix mode
  if (gps_data.fix_status != GPSFixStatus::Fix2D && gps_data.fix_status != GPSFixStatus::Fix3D) {
    SIM7600_LOGW(tag, "GPS not fixed yet");

    // Wait for OK
    status = waitForResponse(AT_OK);
    if (status != Status::Success) return status;

    return Status::GPSNotFixed;
  }

  // Convert latitude and longitude to degrees
  gps_data.latitude =
    (floor(gps_data.latitude / 100) + fmod(gps_data.latitude, 100) / 60) * (n_s == 'N' ? 1 : -1);

  gps_data.longitude =
    (floor(gps_data.longitude / 100) + fmod(gps_data.longitude, 100) / 60) * (e_w == 'E' ? 1 : -1);

  SIM7600_LOGD(tag, "GPS - Lat,Lon: %.6lf,%.6lf", gps_data.latitude, gps_data.longitude);

  return waitForResponse(AT_OK);
}

Status Modem::setNetworkMode(const NetworkMode mode) {
  SIM7600_LOGD(tag, "Setting network mode: %u", static_cast<uint8_t>(mode));

  Status status = sendATCmd("AT+CNMP=%u", static_cast<uint8_t>(mode));
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::getNetworkMode(NetworkMode& mode) {
  SIM7600_LOGD(tag, "Getting network mode");

  Status status = sendATCmdAndWaitResp("AT+CNMP?", "+CNMP:");
  if (status != Status::Success) return status;

  // Expected response: +CNMP: <mode>
  uint8_t mode_code;
  status = parseLine(_rx_buf, 1, "+CNMP: %hhu", &mode_code);
  if (status != Status::Success) return status;

  mode = static_cast<NetworkMode>(mode_code);

  return waitForResponse(AT_OK);
}

Status Modem::waitForNetworkRegistration(RegStatus& reg_status, const uint32_t timeout_ms) {
  SIM7600_LOGI(tag, "Waiting for network registration");

  Status status;
  reg_status = RegStatus::Unknown;

  for (uint32_t start = millis(); (millis() - start) < timeout_ms;) {
    status = getNetworkRegistrationStatus(reg_status);
    if (status != Status::Success) return status;

    // Registered
    if (reg_status == RegStatus::RegisteredHomeNetwork ||
        reg_status == RegStatus::RegisteredRoaming) {
      SIM7600_LOGI(tag,
        "Network registered: %s",
        reg_status == RegStatus::RegisteredHomeNetwork ? "Home Network" : "Roaming");
      return Status::Success;
    }

    // Not yet
    delay(200);
  }

  SIM7600_LOGE(tag, "Timed out waiting for network registration");
  return Status::Timeout;
}

Status Modem::getNetworkRegistrationStatus(RegStatus& reg_status, const uint32_t timeout_ms) {
  SIM7600_LOGD(tag, "Getting network registration status");

  reg_status = RegStatus::Unknown;

  Status status = sendATCmdAndWaitResp("AT+CGREG?", "+CGREG:", timeout_ms);
  if (status != Status::Success) return status;

  // Expected response: +CGREG: <n>,<stat>
  uint8_t n, stat;
  status = parseLine(_rx_buf, 2, "+CGREG: %hhu,%hhu", &n, &stat);
  if (status != Status::Success) return status;

  reg_status = static_cast<RegStatus>(stat);

  // Update internal state
  _current_reg_status = reg_status;
  _registered_on_network =
    (reg_status == RegStatus::RegisteredHomeNetwork || reg_status == RegStatus::RegisteredRoaming);

  return waitForResponse(AT_OK);
}

Status Modem::isRegisteredOnNetwork(bool& registered, const uint32_t timeout_ms) {
  SIM7600_LOGD(tag, "Checking if registered on network");

  registered = false;
  RegStatus reg_status;

  Status status = getNetworkRegistrationStatus(reg_status, timeout_ms);
  if (status != Status::Success) return status;

  registered =
    (reg_status == RegStatus::RegisteredHomeNetwork || reg_status == RegStatus::RegisteredRoaming);

  return Status::Success;
}

bool Modem::isCurrentlyRegisteredOnNetwork() const { return (_registered_on_network); }

RegStatus Modem::getCurrentRegistrationStatus() const { return _current_reg_status; }

Status Modem::configureAPN(const char* apn, const char* user, const char* password) {
  if (apn == nullptr) {
    SIM7600_LOGE(tag, "APN cannot be null");
    return Status::InvalidAPN;
  }

  const char* user_str;
  const char* pass_str;

  if (user == nullptr) {
    user_str = nullptr;
  } else if (strlen(user) == 0) {
    user_str = nullptr;
  } else {
    user_str = user;
  }

  if (password == nullptr) {
    pass_str = nullptr;
  } else if (strlen(password) == 0) {
    pass_str = nullptr;
  } else {
    pass_str = password;
  }

  SIM7600_LOGI(tag,
    "Configuring APN: %s, USER: %s, PASS: %s",
    apn,
    (user_str != nullptr) ? user_str : "-empty-",
    (pass_str != nullptr) ? pass_str : "-empty-");

  Status status;

  // Set authentication if needed
  if (user_str != nullptr && pass_str != nullptr) {
    status = sendATCmd("AT+CGAUTH=1,1,\"%s\",\"%s\"", pass_str, user_str);
    if (status != Status::Success) return status;

    status = waitForResponse(AT_OK);
    if (status != Status::Success) return status;
  }

  // Define context N°1 type IP
  status = sendATCmd("AT+CGDCONT=1,\"IP\",\"%s\"", apn);
  if (status != Status::Success) return status;

  // Wait for OK
  status = waitForResponse(AT_OK);
  if (status != Status::Success) return status;

  SIM7600_LOGI(tag, "APN configured successfully");
  return Status::Success;
}

Status Modem::setNTPServer(const char* ntp_server, const int8_t time_zone) {
  SIM7600_LOGD(tag, "Setting NTP server: %s, time zone: %d", ntp_server, time_zone);

  // Configure NTP server
  Status status = sendATCmd("AT+CNTP=\"%s\",%d", ntp_server, time_zone);
  if (status != Status::Success) return status;

  return waitForResponse(AT_OK);
}

Status Modem::synchronizeTime(NTPSyncStatus& ntp_status, const uint32_t timeout_ms) {
  SIM7600_LOGD(tag, "Synchronizing time via NTP");

  // Sync time
  Status status = sendATCmdAndWaitResp("AT+CNTP", "+CNTP:", timeout_ms);
  if (status != Status::Success) return status;

  // Expected response: +CNTP: <err>
  uint8_t err;
  status = parseLine(_rx_buf, 1, "+CNTP: %hhu", &err);
  if (status != Status::Success) return status;

  ntp_status = static_cast<NTPSyncStatus>(err);
  return Status::Success;
}

Status Modem::getNetworkTime(NTPTimeData& time_data) {
  SIM7600_LOGD(tag, "Getting network time");

  // Clear variables
  time_data = NTPTimeData();

  // Send command to get network time
  Status status = sendATCmdAndWaitResp("AT+CCLK?", "+CCLK:");
  if (status != Status::Success) return status;

  // Expected response: +CCLK: "yy/MM/dd,hh:mm:ss±zz"
  // Example: +CCLK: "08/05/06,14:28:10+32" (May 6, 2008, 14:28:10 GMT+8)
  char time_buffer[21];
  uint8_t parsed = sscanf(_rx_buf, "+CCLK: \"%20[^\"]\"", time_buffer);
  if (parsed != 1) {
    SIM7600_LOGE(tag, "Invalid response");
    return Status::InvalidResponse;
  }

  // Temporary 2 digit year
  uint8_t year;

  // Parse time
  status = parseLine(time_buffer,
    7,
    "%2hhu/%2hhu/%2hhu,%2hhu:%2hhu:%2hhu%2hhd",
    &year,                 // yy (last 2 digits)
    &time_data.month,      // MM
    &time_data.day,        // dd
    &time_data.hour,       // hh
    &time_data.minute,     // mm
    &time_data.second,     // ss
    &time_data.time_zone); // ±zz (quarters)

  if (status != Status::Success) return status;

  time_data.year = 2000 + year; // Convert to full year
  time_data.time_zone /= 4;     // Convert to hours

  return waitForResponse(AT_OK);
}

Status Modem::startTCPIPService() {
  SIM7600_LOGI(tag, "Starting TCP/IP service");

  // Check if TCP/IP service is already running
  bool running  = false;
  Status status = isTCPIPServiceRunning(running);
  if (status != Status::Success) return status;

  if (running) {
    SIM7600_LOGW(tag, "TCP/IP service already running");
    return Status::Success;
  }

  // Send command to start TCP/IP service
  status = sendATCmdAndWaitResp("AT+NETOPEN", "+NETOPEN:", SIM7600_TCP_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected response: +NETOPEN: <err> / where 0=success, others=failure
  uint8_t err;
  status = parseLine(_rx_buf, 1, "+NETOPEN: %hhu", &err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGI(tag, "TCP/IP service started successfully");
    return Status::Success;
  }

  SIM7600_LOGE(tag, "Failed to start TCP/IP service, err: %u", err);
  return Status::Error;
}

Status Modem::stopTCPIPService() {
  SIM7600_LOGI(tag, "Stopping TCP/IP service");

  // Check if TCP/IP service is already stopped
  bool running  = false;
  Status status = isTCPIPServiceRunning(running);
  if (status != Status::Success) return status;

  if (!running) {
    SIM7600_LOGW(tag, "TCP/IP service already stopped");
    return Status::Success;
  }

  // Send command to stop TCP/IP service
  status = sendATCmdAndWaitResp("AT+NETCLOSE", "+NETCLOSE:", SIM7600_TCP_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  // Expected response: +NETCLOSE: <err> / where 0=success, others=failure
  uint8_t err;
  status = parseLine(_rx_buf, 1, "+NETCLOSE: %hhu", &err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGI(tag, "TCP/IP service stopped successfully");
    return Status::Success;
  }

  SIM7600_LOGE(tag, "Failed to stop TCP/IP service, err: %u", err);
  return Status::Error;
}

Status Modem::isTCPIPServiceRunning(bool& running) {
  SIM7600_LOGD(tag, "Checking if TCP/IP service is running");

  running = false;

  Status status = sendATCmdAndWaitResp("AT+NETOPEN?", "+NETOPEN:");
  if (status != Status::Success) return status;

  uint8_t net_opened = false;
  status             = parseLine(_rx_buf, 1, "+NETOPEN: %hhu", &net_opened);
  if (status != Status::Success) return status;

  running = (net_opened == 1);

  return waitForResponse(AT_OK);
}

Status Modem::startMQTTService() {
  SIM7600_LOGI(tag, "Starting MQTT service");

  // Send command to start MQTT service
  Status status = sendATCmd("AT+CMQTTSTART");
  if (status != Status::Success) return status;

  // Expected response: +CMQTTSTART: <err> / where 0=success, 23=already started, others=failure
  uint8_t err;
  status = _waitAsyncMQTTResponse("+CMQTTSTART: ", err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGI(tag, "MQTT service started successfully");
    return Status::Success;
  } else if (err == 23) {
    SIM7600_LOGW(tag, "MQTT service already started");
    return Status::Success;
  }

  SIM7600_LOGE(tag, "Failed to start MQTT service, err: %u", err);
  return Status::Error;
}

Status Modem::stopMQTTService() {
  SIM7600_LOGI(tag, "Stopping MQTT service");

  // Send command to stop MQTT service
  Status status = sendATCmd("AT+CMQTTSTOP");
  if (status != Status::Success) return status;

  // Expected response: +CMQTTSTOP: <err> / where 0=success, 9=already stopped, others=failure
  uint8_t err;
  status = _waitAsyncMQTTResponse("+CMQTTSTOP: ", err);
  if (status != Status::Success) return status;

  if (err == 0) {
    SIM7600_LOGI(tag, "MQTT service stopped successfully");
    return Status::Success;
  } else if (err == 9) {
    SIM7600_LOGW(tag, "MQTT service already stopped");
    return Status::Success;
  }

  SIM7600_LOGE(tag, "Failed to stop MQTT service, err: %u", err);
  return Status::Error;
}

void Modem::loop(const uint32_t duration_ms) {
  if (_serial == nullptr) return;

  // Check for incoming data
  if (_serial->available() == 0) return;

  // readLine processes URCs internally
  readLine(_rx_buf, SIM7600_MODEM_RX_BUFFER_SIZE_B, duration_ms);
}

Status Modem::_writeATCmd() {
  if (_serial == nullptr) return Status::NoSerialPort;

  SIM7600_LOGV(tag, ">> %s", _tx_buf);
  uint16_t len        = strnlen(_tx_buf, SIM7600_MODEM_TX_BUFFER_SIZE_B);
  uint16_t bytes_sent = _serial->write(_tx_buf, len);
  _serial->flush();

  if (bytes_sent < len) {
    SIM7600_LOGE(tag, "Failed to send complete AT command");
    return Status::SerialSentLessThanExpected;
  }

  return Status::Success;
}

Status Modem::_waitPromptAndSendData(const uint8_t* data, const size_t data_length,
  size_t& bytes_sent, const uint32_t timeout_ms) {
  // Wait for prompt
  Status status = waitForPrompt(timeout_ms);
  if (status != Status::Success) return status;

  // Send data
  bytes_sent = _serial->write(data, data_length);
  _serial->flush();

  if (bytes_sent < data_length) {
    SIM7600_LOGE(tag, "Failed to send complete data");
    return Status::SerialSentLessThanExpected;
  }

  return Status::Success;
}

Status Modem::_waitAsyncMQTTResponse(const char* mqtt_response_prefix, uint8_t& err_code) {
  // If success, modem responds with OK and +CMQTT... but order is not guaranteed
  // If error, modem responds with ERROR and +CMQTT... but order is not guaranteed. We need to
  // extract the error code from +CMQTT... response.
  const char* expected_responses[] = {mqtt_response_prefix, AT_OK, AT_ERROR};

  uint8_t found_index = 0;
  Status status =
    waitForResponses(expected_responses, 3, found_index, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
  if (status != Status::Success) return status;

  char format_buf[32];
  snprintf(format_buf, sizeof(format_buf), "%s%%hhu", mqtt_response_prefix);

  bool got_mqtt_first  = (found_index == 0);
  bool got_ok_first    = (found_index == 1);
  bool got_error_first = (found_index == 2);

  // Expected <mqtt_response_prefix>,<err> / where err: 0=success, others=failure
  if (got_mqtt_first) {
    // Parse the error code
    status = parseLine(_rx_buf, 1, format_buf, &err_code);
    if (status != Status::Success) return status;

    // Need to wait for OK or ERROR now
    const char* ok_or_error[] = {AT_OK, AT_ERROR};
    uint8_t ok_error_index    = 0;

    status = waitForResponses(ok_or_error, 2, ok_error_index);
    if (status != Status::Success) return status;

    // If we got ERROR but err_code indicates a recoverable state (e.g. 23=already started,
    // 9=already stopped) we should not treat it as an error
    // The calling function will check err_code and decide how to handle it

  } else if (got_ok_first) {
    // Got OK, now wait for mqtt_response_prefix
    status = waitForResponse(mqtt_response_prefix, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
    if (status != Status::Success) return status;

    status = parseLine(_rx_buf, 1, format_buf, &err_code);
    if (status != Status::Success) return status;

  } else if (got_error_first) {
    // Got ERROR, now wait for mqtt_response_prefix
    status = waitForResponse(mqtt_response_prefix, SIM7600_MQTT_MAX_RESPONSE_TIME_MS);
    if (status != Status::Success) return status;

    status = parseLine(_rx_buf, 1, format_buf, &err_code);
    if (status != Status::Success) return status;
  }

  return Status::Success;
}

bool Modem::_receivedErrorResponse() {
#if SIM7600_LOG_VERBOSE_ERROR_CODES
  // Verbose error response
  if ((strncmp(_rx_buf, "+CME ERROR:", 11) == 0) || (strncmp(_rx_buf, "+CMS ERROR:", 11) == 0)) {
    SIM7600_LOGE(tag, "<< [Verbose Error]: %s", _rx_buf);
    return true;
  }
#endif

  // Error response
  if (strcmp(_rx_buf, AT_ERROR) == 0) {
    SIM7600_LOGW(tag, "Received AT error response");
    return true;
  }

  return false;
}

bool Modem::_handleURCs() {
  // First, check registered handlers (TCPClient, MQTTClient)
  for (uint8_t i = 0; i < _urc_handler_count; i++) {
    if (_urc_handlers[i] == nullptr) continue;

    if (_urc_handlers[i](_rx_buf)) {
      SIM7600_LOGD(tag, "URC handled by registered handler %u", i);
      return true;
    }
  }

  // Then, check modem-specific URCs

  // Network changed
  if (strncmp(_rx_buf, "+CGREG:", 7) == 0) {
    uint8_t n;
    uint8_t stat;
    Status status = parseLine(_rx_buf, 2, "+CGREG: %hhu,%hhu", &n, &stat);

    // If the <n> parameter is present, this is not a URC but a response to AT+CGREG?
    // Exit without further processing
    if (status == Status::Success) {
      SIM7600_LOGD(tag, "CGREG response detected, not a URC");
      return false;
    }

    status = parseLine(_rx_buf, 1, "+CGREG: %hhu", &stat);
    if (status != Status::Success) {
      SIM7600_LOGE(tag, "Failed to parse network registration URC");
      return false;
    }

    RegStatus reg_status = static_cast<RegStatus>(stat);
    bool registered      = (reg_status == RegStatus::RegisteredHomeNetwork ||
                       reg_status == RegStatus::RegisteredRoaming);

    SIM7600_LOGD(tag,
      "URC: Network reg status changed: Registered: %s, status: %u",
      registered ? "YES" : "NO",
      static_cast<uint8_t>(reg_status));

    // Update internal state
    _current_reg_status    = reg_status;
    _registered_on_network = registered;

    if (_cb_network_changed != nullptr) _cb_network_changed(registered, reg_status);

    return true;
  }

  // Modem ready
  if (strcmp(_rx_buf, "RDY") == 0) {
    SIM7600_LOGD(tag, "URC: Modem is ready");

    if (_cb_modem_ready != nullptr) _cb_modem_ready();

    return true;
  }

  // Incoming call
  else if (strcmp(_rx_buf, "RING") == 0) {
    SIM7600_LOGD(tag, "URC: Incoming call detected");
    return true;
  }

  // Network closed unexpectedly
  else if (strcmp(_rx_buf, "+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY") == 0) {
    SIM7600_LOGW(tag, "TCP URC: Network closed unexpectedly");

    if (_cb_tcp_network_closed != nullptr) _cb_tcp_network_closed();

    return true;
  }

  // MQTT no network
  else if (strcmp(_rx_buf, "+CMQTTNONET") == 0) {
    SIM7600_LOGW(tag, "MQTT URC: MQTT no network");

    if (_cb_mqtt_network_closed != nullptr) _cb_mqtt_network_closed();

    return true;
  }

  return false;
}

bool Modem::_registerURCHandler(URCHandler handler) {
  if (handler == nullptr) return false;

  if (_urc_handler_count > MAX_URC_HANDLERS) {
    SIM7600_LOGE(tag, "Cannot register more than %u URC handlers", MAX_URC_HANDLERS);
    return false;
  }

  // Check if already registered
  for (uint8_t i = 0; i < _urc_handler_count; i++) {
    if (_urc_handlers[i] == handler) {
      SIM7600_LOGW(tag, "URC handler already registered");
      return false;
    }
  }

  _urc_handlers[_urc_handler_count++] = handler;
  SIM7600_LOGD(tag, "URC handler registered. Total %u", _urc_handler_count);
  return true;
}

} // namespace SIM7600