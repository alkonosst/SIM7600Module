/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>

namespace SIM7600 {

template <typename T, uint8_t MAX_CLIENTS>
class ClientManager {
  public:
  uint8_t getClientID() const { return _client_id; }
  uint8_t getClientCount() const { return _clients_used; }
  bool isUsable() const { return _client_usable; }

  protected:
  static uint8_t _clients_used;
  static T* _instances[MAX_CLIENTS];

  uint8_t _client_id;
  bool _client_usable;

  ClientManager()
      : _client_id(0)
      , _client_usable(true) {
    if (_clients_used >= MAX_CLIENTS) {
      _client_usable = false;
      return;
    }

    _client_id             = _clients_used++;
    _instances[_client_id] = static_cast<T*>(this);
  }

  ~ClientManager() {
    if (_client_usable) {
      _instances[_client_id] = nullptr;
      _clients_used--;
    }
  }
};

template <typename T, uint8_t MAX_CLIENTS>
uint8_t ClientManager<T, MAX_CLIENTS>::_clients_used = 0;

template <typename T, uint8_t MAX_CLIENTS>
T* ClientManager<T, MAX_CLIENTS>::_instances[MAX_CLIENTS]{};

} // namespace SIM7600