#ifndef LOG_H
#define LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include "common.h"

// Log levels
typedef enum {
    LOG_ERROR = 0,  // Error conditions
    LOG_WARN,       // Warning conditions
    LOG_INFO,       // Informational messages
    LOG_DEBUG,      // Debug messages
    LOG_TRACE       // Detailed tracing messages
} LogLevel;

// Log categories
typedef enum {
    LOG_CORE = 0,       // Core system messages
    LOG_BUS,            // Message bus
    LOG_FLIGHT_CTRL,    // Flight controller
    LOG_AUTOPILOT,      // Autopilot
    LOG_GPS,            // GPS receiver
    LOG_INS,            // INS
    LOG_LANDING,        // Landing radio
    LOG_SATCOM,         // Satellite communications
    LOG_NUM_CATEGORIES  // Must be last
} LogCategory;

// Initialize logging system
void log_init(void);

// Clean up logging system
void log_cleanup(void);

// Set global log level
void log_set_level(LogLevel level);

// Set category-specific log level
void log_set_category_level(LogCategory category, LogLevel level);

// Enable/disable category
void log_enable_category(LogCategory category, bool enable);

// Main logging function
void log_write(LogCategory category, LogLevel level, const char* file, int line, 
               const char* func, const char* fmt, ...);

// Convenience macros
#define LOG_ERROR(cat, ...) log_write(cat, LOG_ERROR, __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_WARN(cat, ...)  log_write(cat, LOG_WARN,  __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_INFO(cat, ...)  log_write(cat, LOG_INFO,  __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_DEBUG(cat, ...) log_write(cat, LOG_DEBUG, __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_TRACE(cat, ...) log_write(cat, LOG_TRACE, __FILE__, __LINE__, __func__, __VA_ARGS__)

// String conversion utilities
const char* log_level_to_string(LogLevel level);
const char* log_category_to_string(LogCategory category);

#endif // LOG_H
