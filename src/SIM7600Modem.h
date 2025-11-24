/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "SIM7600Common.h"

namespace SIM7600 {

/// @brief SIM card status.
enum class SIMStatus { Ready = 0, PinWaiting, Error };

/// @brief Network mode (3G, 4G, etc.). Default is Automatic.
enum class NetworkMode {
  Automatic                        = 2,
  GSM_Only                         = 13,
  WCDMA_Only                       = 14,
  LTE_Only                         = 38,
  TDS_CDMA_Only                    = 59,
  CDMA_Only                        = 9,
  EVDO_Only                        = 10,
  GSM_WCDMA_Only                   = 19,
  CDMA_EVDO_Only                   = 22,
  Any_But_LTE                      = 48,
  GSM_TDSCDMA_Only                 = 60,
  GSM_WCDMA_TDSCDMA_Only           = 63,
  CDMA_EVDO_GSM_WCDMA_TDSCDMA_Only = 67,
  GSM_WCDMA_LTE_Only               = 39,
  GSM_LTE_Only                     = 51,
  WCDMA_LTE_Only                   = 54
};

/// @brief Network registration status.
enum class RegStatus {
  NotRegisteredAndNotSearching = 0,
  RegisteredHomeNetwork        = 1,
  NotRegisteredButSearching    = 2,
  RegistrationDenied           = 3,
  Unknown                      = 4,
  RegisteredRoaming            = 5
};

/// @brief NTP synchronization status.
enum class NTPSyncStatus {
  Success                    = 0,
  UnknownError               = 1,
  WrongParameter             = 2,
  WrongDateAndTimeCalculated = 3,
  NetworkError               = 4,
  TimeZoneError              = 5,
  TimeoutError               = 6
};

/// @brief GPS fix status.
enum class GPSFixStatus { NoFix = 0, Fix2D = 2, Fix3D = 3 };

/// @brief NTP time data structure.
struct NTPTimeData {
  uint8_t day      = 1;
  uint8_t month    = 1;
  uint16_t year    = 2000;
  uint8_t hour     = 0;
  uint8_t minute   = 0;
  uint8_t second   = 0;
  int8_t time_zone = 0; // Hours from UTC
};

/// @brief GNSS data structure.
struct GPSData {
  // Position
  double latitude  = 0.0; // Degrees
  double longitude = 0.0; // Degrees
  float speed      = 0.0; // Knots
  float course     = 0.0; // Degrees
  float altitude   = 0.0; // Meters above sea level

  // Time (UTC)
  uint8_t day    = 1;
  uint8_t month  = 1;
  uint16_t year  = 2000;
  uint8_t hour   = 0;
  uint8_t minute = 0;
  uint8_t second = 0;

  // Quality indicators
  GPSFixStatus fix_status    = GPSFixStatus::NoFix; // Fix status
  uint8_t gps_satellites     = 0;                   // Number of GPS satellites used
  uint8_t glonass_satellites = 0;                   // Number of GLONASS satellites used
  uint8_t beidou_satellites  = 0;                   // Number of BeiDou satellites used
  float hdop                 = 0.0;                 // Horizontal dilution of precision
  float pdop                 = 0.0;                 // Position dilution of precision
  float vdop                 = 0.0;                 // Vertical dilution of precision
};

// Forward declaration
class TCPClient;
class MQTTClient;

/// @brief SIM7600 modem control class.
class Modem {
  public:
  // Callback function types. Use std::function if available.
#if SIM7600_HAS_STD_FUNCTION
  using ModemReadyCB        = std::function<void()>;
  using NetworkChangedCB    = std::function<void(const bool registered, const RegStatus status)>;
  using TCPNetworkClosedCB  = std::function<void()>;
  using MQTTNetworkClosedCB = std::function<void()>;
#else
  using ModemReadyCB        = void (*)();
  using NetworkChangedCB    = void (*)(const bool registered, const RegStatus status);
  using TCPNetworkClosedCB  = void (*)();
  using MQTTNetworkClosedCB = void (*)();
#endif

  /**
   * @brief Create a new Modem instance. You need to set the serial port later using
   * setSerialPort().
   */
  Modem();

  /**
   * @brief Create a new Modem instance with the given serial port.
   * @param serial Serial port.
   */
  Modem(Stream* serial);

  /**
   * @brief Set the Serial Port object to be used by the modem.
   * @param serial Serial port.
   */
  void setSerialPort(Stream* serial);

  /**
   * @brief Set the modem ready callback.
   * @param callback Callback function when the modem delivers ready status ("RDY" URC).
   * @return Status::Success on success, error code otherwise.
   */
  Status setModemReadyCallback(ModemReadyCB callback);

  /**
   * @brief Set the network changed callback.
   * @param callback Callback function to be called when the network registration status changes.
   * @return Status::Success on success, error code otherwise.
   */
  Status setNetworkChangedCallback(NetworkChangedCB callback);

  /**
   * @brief Set the TCP network closed callback.
   * @param callback Callback function to be called when the TCP network is closed.
   * @return Status::Success on success, error code otherwise.
   */
  Status setTCPNetworkClosedCallback(TCPNetworkClosedCB callback);

  /**
   * @brief Set the MQTT network closed callback.
   * @param callback Callback function to be called when the MQTT network is closed.
   * @return Status::Success on success, error code otherwise.
   */
  Status setMQTTNetworkClosedCallback(MQTTNetworkClosedCB callback);

  /* ----------------------------------- Modem control methods ---------------------------------- */

  /**
   * @brief Initialize the modem.
   * @param pin PIN code for SIM card, if required.
   * @param timeout_ms Timeout for testing AT communication [ms].
   * @return Status::Success on success, error code otherwise.
   */
  Status init(const char* pin = nullptr,
    uint32_t timeout_ms       = SIM7600_MODEM_DEFAULT_INIT_TIMEOUT_MS);

  /**
   * @brief Reset the modem. The modem losses its configuration and needs to be re-initialized
   * with init() in the next power on.
   * @return Status::Success on success, error code otherwise.
   */
  Status reset();

  /**
   * @brief Power off the modem. The modem losses its configuration and needs to be re-initialized
   * with init() in the next power on.
   * @return Status::Success on success, error code otherwise.
   */
  Status powerOff();

  /**
   * @brief Send an AT command to the modem.
   * @param cmd_format Format string for the AT command.
   * @param ... Additional arguments for the format string.
   * @return Status::Success on success, error code otherwise.
   */
  Status sendATCmd(const char* cmd_format, ...);

  /**
   * @brief Send an AT command to the modem and wait for a specific response.
   * @param cmd AT command to send.
   * @param expected_response Expected response string.
   * @param timeout_ms Timeout in milliseconds to wait for the response.
   * @return Status::Success on success, error code otherwise.
   */
  Status sendATCmdAndWaitResp(const char* cmd, const char* expected_response,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Send an AT command to the modem and wait for one of multiple expected responses.
   * @param cmd AT command to send.
   * @param expected_responses Array of expected response strings.
   * @param response_count Number of expected responses.
   * @param found_index Index of the found response in the expected_responses array.
   * @param timeout_ms Timeout in milliseconds to wait for the response.
   * @return Status::Success on success, error code otherwise.
   */
  Status sendATCmdAndWaitResp(const char* cmd, const char** expected_responses,
    const uint8_t response_count, uint8_t& found_index,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Wait for a specific response from the modem.
   * @param expected_response Expected response string.
   * @param timeout_ms Timeout in milliseconds to wait for the response.
   * @return Status::Success on success, error code otherwise.
   */
  Status waitForResponse(const char* expected_response,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Wait for one of multiple expected responses from the modem.
   * @param expected_responses Array of expected response strings.
   * @param response_count Number of expected responses.
   * @param found_index Index of the found response in the expected_responses array.
   * @param timeout_ms Timeout in milliseconds to wait for the response.
   * @return Status::Success on success, error code otherwise.
   */
  Status waitForResponses(const char** expected_responses, const uint8_t response_count,
    uint8_t& found_index, const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Wait for the '>' prompt from the modem.
   * @param timeout_ms Timeout in milliseconds to wait for the prompt.
   * @return Status::Success on success, error code otherwise.
   */
  Status waitForPrompt(const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Read a line from the modem. A line is terminated by \r\n.
   * @param buffer Buffer to store the read line.
   * @param buffer_size Size of the buffer.
   * @param timeout_ms Timeout in milliseconds to wait for the line.
   * @return Status::Success on success, error code otherwise.
   */
  Status readLine(char* const buffer, const size_t buffer_size,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Read a specific number of bytes from the modem.
   * @param buffer Buffer to store the read bytes.
   * @param buffer_size Size of the buffer.
   * @param bytes_to_read Number of bytes to read.
   * @param bytes_read Number of bytes actually read.
   * @param timeout_ms Timeout in milliseconds to wait for the bytes.
   * @return Status::Success on success, error code otherwise.
   */
  Status readBytes(uint8_t* const buffer, const size_t buffer_size, const size_t bytes_to_read,
    size_t& bytes_read, const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Parse a line with the given format.
   * @param buffer Line buffer to parse.
   * @param parameters Number of parameters to parse.
   * @param format Format string.
   * @param ... Additional arguments for the format string.
   * @return Status::Success on success, error code otherwise.
   */
  Status parseLine(char* const buffer, const uint8_t parameters, const char* format, ...);

  /* -------------------------------- Status and control methods -------------------------------- */

  /**
   * @brief Modem main loop. Should be called periodically to handle URCs.
   * @param duration_ms Duration in milliseconds to run the loop.
   * @note This method exits inmediately if no data is available on the serial port, or the serial
   * port is invalid.
   */
  void loop(const uint32_t duration_ms = SIM7600_MODEM_DEFAULT_LOOP_DURATION_MS);

  /**
   * @brief Test communication with the modem by sending an AT command.
   * @param timeout_ms Timeout in milliseconds to wait for the response.
   * @return Status::Success on success, error code otherwise.
   */
  Status testAT(const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Get the SIM card status.
   * @param sim_status SIMStatus enum to store the SIM card status.
   * @return Status::Success on success, error code otherwise.
   */
  Status getSIMStatus(SIMStatus& sim_status);

  /**
   * @brief Unlock the SIM card with the given PIN code.
   * @param pin PIN code.
   * @return Status::Success on success, error code otherwise.
   */
  Status unlockSIM(const char* pin);

  /**
   * @brief Get the signal quality: RSSI and BER (bit error rate).
   * @param rssi Reference to store the RSSI value in dBm.
   * @param ber_min Reference to store the minimum BER value.
   * @param ber_max Reference to store the maximum BER value.
   * @return Status::Success on success, error code otherwise.
   */
  Status getSignalQuality(float& rssi, float& ber_min, float& ber_max);

  /**
   * @brief Disable SMS notifications.
   * @return Status::Success on success, error code otherwise.
   */
  Status disableSMSNotifications();

  /* ---------------------------------------- GPS methods --------------------------------------- */

  /**
   * @brief Set the GPS antenna voltage.
   * @param voltage_mv Voltage in millivolts (e.g., 1800 for 1.8V, 2800 for 2.8V).
   * @return Status::Success on success, error code otherwise.
   */
  Status setGPSAntennaVoltage(const uint16_t voltage_mv);

  /**
   * @brief Get the GPS antenna voltage.
   * @param voltage_mv Reference to store the voltage in millivolts.
   * @return Status::Success on success, error code otherwise.
   */
  Status getGPSAntennaVoltage(uint16_t& voltage_mv);

  /**
   * @brief Enable the GPS antenna voltage.
   * @return Status::Success on success, error code otherwise.
   */
  Status enableGPSAntennaVoltage();

  /**
   * @brief Disable the GPS antenna voltage.
   * @return Status::Success on success, error code otherwise.
   */
  Status disableGPSAntennaVoltage();

  /**
   * @brief Check if the GPS antenna voltage is enabled.
   * @param enabled Reference to store the enabled status.
   * @return Status::Success on success, error code otherwise.
   */
  Status isGPSAntennaVoltageEnabled(bool& enabled);

  /**
   * @brief Enable the GPS.
   * @return Status::Success on success, error code otherwise.
   */
  Status enableGPS();

  /**
   * @brief Disable the GPS.
   * @return Status::Success on success, error code otherwise.
   */
  Status disableGPS();

  /**
   * @brief Check if the GPS is enabled.
   * @param enabled Reference to store the enabled status.
   * @return Status::Success on success, error code otherwise.
   */
  Status isGPSEnabled(bool& enabled);

  /**
   * @brief Enable or disable GPS auto start on modem power up.
   * @param enable True to enable, false to disable.
   * @return Status::Success on success, error code otherwise.
   */
  Status enableGPSAutoStart(const bool enable);

  /**
   * @brief Get the GPS auto start status.
   * @param enabled Reference to store the enabled status.
   * @return Status::Success on success, error code otherwise.
   */
  Status getGPSAutoStart(bool& enabled);

  /**
   * @brief Get the current GPS data.
   * @param gps_data Reference to store the GPS data.
   * @return Status::Success on success, error code otherwise.
   */
  Status getGPSData(GPSData& gps_data);

  /* -------------------------------- Network management methods -------------------------------- */

  /**
   * @brief Set the network mode.
   * @param mode NetworkMode enum value.
   * @return Status::Success on success, error code otherwise.
   */
  Status setNetworkMode(const NetworkMode mode);

  /**
   * @brief Get the current network mode.
   * @param mode Reference to store the NetworkMode enum value.
   * @return Status::Success on success, error code otherwise.
   */
  Status getNetworkMode(NetworkMode& mode);

  /**
   * @brief Wait for network registration.
   * @param reg_status Reference to store the RegStatus enum value.
   * @param timeout_ms Timeout in milliseconds to wait for registration.
   * @return Status::Success on success, error code otherwise.
   */
  Status waitForNetworkRegistration(RegStatus& reg_status,
    const uint32_t timeout_ms = SIM7600_MODEM_NETWORK_TIMEOUT_MS);

  /**
   * @brief Check if the modem is registered on the network.
   * @param reg_status Reference to store the RegStatus enum value.
   * @param timeout_ms Timeout in milliseconds to wait for registration.
   * @return Status::Success on success, error code otherwise.
   */
  Status getNetworkRegistrationStatus(RegStatus& reg_status,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Check if the modem is registered on the network.
   * @param registered Reference to store the registered status.
   * @param timeout_ms Timeout in milliseconds to wait for registration.
   * @return Status::Success on success, error code otherwise.
   */
  Status isRegisteredOnNetwork(bool& registered,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Check if the modem is currently registered on the network. This method returns the last
   * known registration status without querying the modem. It is updated when receiving the +CGREG
   * URC and when querying the registration status.
   * @return true if registered on the network, false otherwise.
   */
  bool isCurrentlyRegisteredOnNetwork() const;

  /**
   * @brief Get the last known registration status without querying the modem. It is updated when
   * receiving the +CGREG URC and when querying the registration status.
   * @return RegStatus Registration status.
   */
  RegStatus getCurrentRegistrationStatus() const;

  /**
   * @brief Configure the APN settings for GPRS connection.
   * @param apn Access Point Name. Cannot be nullptr or an empty string.
   * @param user Username for APN, if required. Can be nullptr or an empty string.
   * @param password Password for APN, if required. Can be nullptr or an empty string.
   * @return Status::Success on success, error code otherwise.
   */
  Status configureAPN(const char* apn, const char* user = nullptr, const char* password = nullptr);

  /* ---------------------------------------- NTP methods --------------------------------------- */

  /**
   * @brief Set the NTP server to synchronize time.
   * @param ntp_server NTP server address.
   * @param time_zone Time zone offset from UTC in hours.
   * @return Status::Success on success, error code otherwise.
   */
  Status setNTPServer(const char* ntp_server, const int8_t time_zone = 0);

  /**
   * @brief Synchronize time with the configured NTP server. You must set the NTP server. At least
   * one successful synchronization is required to get the network time.
   * @param ntp_status Reference to store the NTPSyncStatus enum value.
   * @param timeout_ms Timeout in milliseconds to wait for synchronization.
   * @return Status::Success on success, error code otherwise.
   */
  Status synchronizeTime(NTPSyncStatus& ntp_status,
    const uint32_t timeout_ms = SIM7600_MODEM_NTP_SYNC_TIMEOUT_MS);

  /**
   * @brief Get the current network time.
   * @param time_data Reference to store the NTPTimeData structure.
   * @return Status::Success on success, error code otherwise.
   */
  Status getNetworkTime(NTPTimeData& time_data);

  /* ---------------------------------- TCP/IP service methods ---------------------------------- */

  /**
   * @brief Start the TCP/IP service.
   * @return Status::Success on success, error code otherwise.
   */
  Status startTCPIPService();

  /**
   * @brief Stop the TCP/IP service.
   * @return Status::Success on success, error code otherwise.
   */
  Status stopTCPIPService();

  /**
   * @brief Check if the TCP/IP service is running.
   * @param running Reference to store the running status.
   * @return Status::Success on success, error code otherwise.
   */
  Status isTCPIPServiceRunning(bool& running);

  /* ----------------------------------- MQTT service methods ----------------------------------- */

  /**
   * @brief Start the MQTT service.
   * @return Status::Success on success, error code otherwise.
   */
  Status startMQTTService();

  /**
   * @brief Stop the MQTT service.
   * @return Status::Success on success, error code otherwise.
   */
  Status stopMQTTService();

  private:
  Stream* _serial;
  ModemReadyCB _cb_modem_ready;
  NetworkChangedCB _cb_network_changed;
  TCPNetworkClosedCB _cb_tcp_network_closed;
  MQTTNetworkClosedCB _cb_mqtt_network_closed;
  bool _registered_on_network;
  RegStatus _current_reg_status;

  char _tx_buf[SIM7600_MODEM_TX_BUFFER_SIZE_B];
  char _rx_buf[SIM7600_MODEM_RX_BUFFER_SIZE_B];

  Status _writeATCmd();
  Status _waitPromptAndSendData(const uint8_t* data, const size_t data_length, size_t& bytes_sent,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);
  Status _waitAsyncMQTTResponse(const char* mqtt_response_prefix, uint8_t& err_code);
  bool _receivedErrorResponse();

  bool _handleURCs();

  // URC handler registration
  using URCHandler = bool (*)(const char* line);

  static constexpr uint8_t MAX_URC_HANDLERS = 2;
  URCHandler _urc_handlers[MAX_URC_HANDLERS];
  uint8_t _urc_handler_count;

  bool _registerURCHandler(URCHandler handler);

  friend class TCPClient;
  friend class MQTTClient;
};

} // namespace SIM7600