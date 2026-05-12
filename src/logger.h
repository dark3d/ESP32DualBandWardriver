#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "configs.h"
#include "utils.h"

#define STD_MSG  1
#define WARN_MSG 2
#define GUD_MSG  3

// ============================================================
// Web log ring buffer — captures the last LOG_RING_SIZE lines
// so they can be served via the /log web endpoint.
// Sized to keep RAM impact minimal (~8KB for 100 x 80 chars).
// ============================================================
#define LOG_RING_SIZE   100
#define LOG_LINE_MAX    120  // max chars per line including prefix

class Logger {
public:
  static void log(uint8_t type, String msg);

  // Ring buffer — public so serveConfigPage() can read it directly
  static String ring[LOG_RING_SIZE];
  static int    ring_head;   // index of next write slot
  static int    ring_count;  // how many slots are filled (0..LOG_RING_SIZE)
};

#endif
