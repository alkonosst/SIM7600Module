/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>

#include "SIM7600Log.h"

#ifndef SIM7600_LOG_CUSTOM
void sim7600Log(char level, const char* tag, const char* format, ...) {
  // Create buffer for formatted message
  char buffer[SIM7600_LOG_BUFFER_SIZE_B];

  // Format the message with variable arguments
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

// Print with level and tag
#if defined(ESP32) || defined(ESP8266)
  // ESP platforms support printf formatting
  Serial.printf("[%c][%s] %s\n", level, tag, buffer);
#else
  // Other platforms
  Serial.print('[');
  Serial.print(level);
  Serial.print("][");
  Serial.print(tag);
  Serial.print("] ");
  Serial.println(buffer);
#endif
}
#endif // SIM7600_LOG_CUSTOM