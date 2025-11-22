/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "SIM7600ClientManager.h"
#include "SIM7600Common.h"
#include "SIM7600Modem.h"

namespace SIM7600 {

/// @brief TCP socket close reason.
enum class TCPCloseReason { ByLocal_Active = 0, ByRemote_Passive, ForSendingTimeout };

/// @brief TCP Client class.
class TCPClient : public Client, public ClientManager<TCPClient, SIM7600_TCP_MAX_CLIENTS> {
  public:
// Callback function types. Use std::function if available.
#if SIM7600_HAS_STD_FUNCTION
  using DataReceivedCB = std::function<void(TCPClient* const client)>;
  using ConnectionClosedCB =
    std::function<void(TCPClient* const client, const TCPCloseReason reason)>;
#else
  using DataReceivedCB     = void (*)(TCPClient* const client);
  using ConnectionClosedCB = void (*)(TCPClient* const client, const TCPCloseReason reason);
#endif

  /**
   * @brief Create a new TCPClient instance. You need to set the modem later using setModem().
   */
  TCPClient();

  /**
   * @brief Create a new TCPClient instance with the given modem.
   * @param modem Modem instance.
   */
  TCPClient(Modem* modem);

  /**
   * @brief Destroy the TCPClient instance.
   */
  ~TCPClient();

  /**
   * @brief Set the Modem instance to be used by the TCPClient.
   * @param modem Modem instance.
   * @return Status::Success on success, error code otherwise.
   */
  Status setModem(Modem* const modem);

  /**
   * @brief Set the data received callback.
   * @param callback Callback function to be called when data is received.
   * @return Status::Success on success, error code otherwise.
   */
  Status setDataReceivedCallback(DataReceivedCB callback);
  /**
   * @brief Set the connection closed callback.
   * @param callback Callback function to be called when the connection is closed.
   * @return Status::Success on success, error code otherwise.
   */
  Status setConnectionClosedCallback(ConnectionClosedCB callback);

  /**
   * @brief Connect to a host using IP address and port.
   * @param ip IP address.
   * @param port Port number.
   * @return Status::Success on success, error code otherwise.
   */
  Status connectToHost(const IPAddress ip, const uint16_t port);

  /**
   * @brief Connect to a host using hostname and port.
   * @param host Hostname.
   * @param port Port number.
   * @return Status::Success on success, error code otherwise.
   */
  Status connectToHost(const char* host, const uint16_t port);

  /**
   * @brief Disconnect from the host.
   * @return Status::Success on success, error code otherwise.
   */
  Status disconnect();

  /**
   * @brief Check if the client is connected to the host.
   * @param connected Reference to store the connected status.
   * @return Status::Success on success, error code otherwise.
   */
  Status isConnected(bool& connected);

  /**
   * @brief Send data to the connected host.
   * @param buffer Data buffer to send.
   * @param buffer_size Size of the data buffer.
   * @param bytes_sent Reference to store the number of bytes sent.
   * @return Status::Success on success, error code otherwise.
   */
  Status sendData(const uint8_t* buffer, const size_t buffer_size, size_t& bytes_sent);

  /**
   * @brief Read data from the connected host.
   * @param buffer Buffer to store the read data.
   * @param buffer_size Size of the buffer.
   * @param bytes_read Reference to store the number of bytes read.
   * @param timeout_ms Timeout in milliseconds to wait for data.
   * @return Status::Success on success, error code otherwise.
   */
  Status readData(uint8_t* buffer, const size_t buffer_size, size_t& bytes_read,
    const uint32_t timeout_ms = SIM7600_MODEM_DEFAULT_TIMEOUT_MS);

  /**
   * @brief Get the number of available bytes to read.
   * @param available_bytes Reference to store the number of available bytes.
   * @return Status::Success on success, error code otherwise.
   */
  Status getAvailableBytes(size_t& available_bytes);

  // Overridden methods from Client
  int connect(IPAddress ip, uint16_t port);
  int connect(const char* host, uint16_t port);
  size_t write(uint8_t c);
  size_t write(const uint8_t* buf, size_t size);
  int available();
  int read();
  int read(uint8_t* buf, size_t size);
  int peek();
  void flush();
  void stop();
  uint8_t connected();
  operator bool();

  private:
  Modem* _modem;
  bool _connected;
  size_t _available_bytes;
  uint8_t _peek_byte;
  bool _peek_available;

  char* _tx_buf;
  char* _rx_buf;

  DataReceivedCB _cb_data;
  ConnectionClosedCB _cb_connection_closed;

  static bool _handleTCPURCs(const char* line);
};

} // namespace SIM7600