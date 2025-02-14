#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

// Log configuration
static struct {
    LogLevel global_level;
    LogLevel category_levels[LOG_NUM_CATEGORIES];
    bool category_enabled[LOG_NUM_CATEGORIES];
    pthread_mutex_t mutex;
    FILE* log_file;
} log_config;

// Level strings
static const char* level_strings[] = {
    "ERROR",
    "WARN ",
    "INFO ",
    "DEBUG",
    "TRACE"
};

// Category strings
static const char* category_strings[] = {
    "CORE",
    "BUS",
    "FCTL",
    "AUTO",
    "GPS",
    "INS",
    "LAND",
    "SAT"
};

void log_init(void) {
    // Initialize mutex
    pthread_mutex_init(&log_config.mutex, NULL);
    
    // Set default levels
    log_config.global_level = LOG_INFO;
    for (int i = 0; i < LOG_NUM_CATEGORIES; i++) {
        log_config.category_levels[i] = LOG_INFO;
        log_config.category_enabled[i] = true;
    }
    
    // Open log file - use timestamp in filename
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    char filename[64];
    strftime(filename, sizeof(filename), "airplane_sim_%Y%m%d_%H%M%S.log", tm_info);
    
    log_config.log_file = fopen(filename, "w");
    if (!log_config.log_file) {
        fprintf(stderr, "Failed to open log file %s\n", filename);
        log_config.log_file = stderr;
    }
    
    LOG_INFO(LOG_CORE, "Logging system initialized");
}

void log_cleanup(void) {
    pthread_mutex_lock(&log_config.mutex);
    if (log_config.log_file && log_config.log_file != stderr) {
        fclose(log_config.log_file);
    }
    pthread_mutex_unlock(&log_config.mutex);
    pthread_mutex_destroy(&log_config.mutex);
}

void log_set_level(LogLevel level) {
    pthread_mutex_lock(&log_config.mutex);
    log_config.global_level = level;
    pthread_mutex_unlock(&log_config.mutex);
    LOG_INFO(LOG_CORE, "Global log level set to %s", level_strings[level]);
}

void log_set_category_level(LogCategory category, LogLevel level) {
    if (category >= LOG_NUM_CATEGORIES) return;
    
    pthread_mutex_lock(&log_config.mutex);
    log_config.category_levels[category] = level;
    pthread_mutex_unlock(&log_config.mutex);
    
    LOG_INFO(LOG_CORE, "Category %s log level set to %s", 
             category_strings[category], level_strings[level]);
}

void log_enable_category(LogCategory category, bool enable) {
    if (category >= LOG_NUM_CATEGORIES) return;
    
    pthread_mutex_lock(&log_config.mutex);
    log_config.category_enabled[category] = enable;
    pthread_mutex_unlock(&log_config.mutex);
    
    LOG_INFO(LOG_CORE, "Category %s %s", 
             category_strings[category], enable ? "enabled" : "disabled");
}

void log_write(LogCategory category, LogLevel level, const char* file, int line,
               const char* func, const char* fmt, ...) {
    if (category >= LOG_NUM_CATEGORIES) return;
    
    pthread_mutex_lock(&log_config.mutex);
    
    // Check if we should log this message
    if (!log_config.category_enabled[category] ||
        level > log_config.global_level ||
        level > log_config.category_levels[category]) {
        pthread_mutex_unlock(&log_config.mutex);
        return;
    }
    
    // Get current time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm* tm_info = localtime(&ts.tv_sec);
    
    // Write timestamp and log level
    fprintf(log_config.log_file, "%02d:%02d:%02d.%03ld %-5s %-4s ",
            tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
            ts.tv_nsec / 1000000,
            level_strings[level],
            category_strings[category]);
    
    // Write file and line for debug/trace levels
    if (level >= LOG_DEBUG) {
        fprintf(log_config.log_file, "[%s:%d %s] ",
                file, line, func);
    }
    
    // Write the actual message
    va_list args;
    va_start(args, fmt);
    vfprintf(log_config.log_file, fmt, args);
    va_end(args);
    fprintf(log_config.log_file, "\n");
    
    // Flush buffer for important messages
    if (level <= LOG_WARN) {
        fflush(log_config.log_file);
    }
    
    pthread_mutex_unlock(&log_config.mutex);
}

const char* log_level_to_string(LogLevel level) {
    if (level >= LOG_ERROR && level <= LOG_TRACE) {
        return level_strings[level];
    }
    return "UNKNOWN";
}

const char* log_category_to_string(LogCategory category) {
    if (category < LOG_NUM_CATEGORIES) {
        return category_strings[category];
    }
    return "UNKNOWN";
}
