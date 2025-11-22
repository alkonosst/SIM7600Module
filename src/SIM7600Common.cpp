/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "SIM7600Common.h"

const char* SIM7600::statusToString(SIM7600::Status status) {
#define X(name) #name,
  static const char* status_strings[] = {SIM7600_STATUS_LIST};
#undef X

  return status_strings[static_cast<uint8_t>(status)];
}