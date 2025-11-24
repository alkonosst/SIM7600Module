/**
 * SPDX-FileCopyrightText: 2025 Maximiliano Ramirez <maximiliano.ramirezbravo@gmail.com>
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

// Log levels
#define SIM7600_LOG_LEVEL_NONE    0
#define SIM7600_LOG_LEVEL_ERROR   1
#define SIM7600_LOG_LEVEL_WARNING 2
#define SIM7600_LOG_LEVEL_INFO    3
#define SIM7600_LOG_LEVEL_DEBUG   4
#define SIM7600_LOG_LEVEL_VERBOSE 5

// Set default log level
#ifndef SIM7600_LOG_LEVEL
#define SIM7600_LOG_LEVEL SIM7600_LOG_LEVEL_NONE
#endif

// Set default stream
#ifndef SIM7600_LOG_STREAM
#define SIM7600_LOG_STREAM Serial
#endif

// Enable verbose error codes, only printed when log level is ERROR or higher
#ifndef SIM7600_LOG_VERBOSE_ERROR_CODES
#define SIM7600_LOG_VERBOSE_ERROR_CODES 0
#endif

// Log buffer size
#ifndef SIM7600_LOG_BUFFER_SIZE_B
#define SIM7600_LOG_BUFFER_SIZE_B 256
#endif

// Check if user wants to use ESP32 native logs
#if defined(ESP32) && defined(SIM7600_USE_ESP32_LOGS)
#include <esp_log.h>

// Map SIM7600 logs directly to ESP32 logs
#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_ERROR
#define SIM7600_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGE(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_WARNING
#define SIM7600_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGW(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_INFO
#define SIM7600_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGI(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_DEBUG
#define SIM7600_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGD(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_VERBOSE
#define SIM7600_LOGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGV(tag, format, ...) ((void)0)
#endif

#else
// Use custom or default log function
#ifndef SIM7600_LOG_CUSTOM
// Use default log function
void sim7600Log(char level, const char* tag, const char* format, ...);
#define SIM7600_LOG_FUNCTION sim7600Log
#else
#define SIM7600_LOG_FUNCTION SIM7600_LOG_CUSTOM
#endif

// Log macros
#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_ERROR
#define SIM7600_LOGE(tag, format, ...) SIM7600_LOG_FUNCTION('E', tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGE(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_WARNING
#define SIM7600_LOGW(tag, format, ...) SIM7600_LOG_FUNCTION('W', tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGW(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_INFO
#define SIM7600_LOGI(tag, format, ...) SIM7600_LOG_FUNCTION('I', tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGI(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_DEBUG
#define SIM7600_LOGD(tag, format, ...) SIM7600_LOG_FUNCTION('D', tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGD(tag, format, ...) ((void)0)
#endif

#if SIM7600_LOG_LEVEL >= SIM7600_LOG_LEVEL_VERBOSE
#define SIM7600_LOGV(tag, format, ...) SIM7600_LOG_FUNCTION('V', tag, format, ##__VA_ARGS__)
#else
#define SIM7600_LOGV(tag, format, ...) ((void)0)
#endif

#endif // SIM7600_USE_ESP32_LOGS