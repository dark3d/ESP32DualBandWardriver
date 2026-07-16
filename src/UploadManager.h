// UploadManager.h — strangler piece #1
//
// Owns the "push pending SD logs to N services" orchestration, lifted out of
// WiFiOps. Each service (WiGLE, WDG, ...) is a data-registered UploadService
// whose actual TLS upload is a callback (initially wrapping the existing
// wigleUpload/wdgwarsUpload; later strangler steps move framing in here).
//
// Value delivered by step 1: per-(file x service) state + per-service
// exponential backoff and a min interval, so one service throttling never
// stalls the dock and we structurally cannot machine-gun an endpoint.
//
// Blocking-but-paced: TLS is blocking on the C5, so runNext() performs at most
// one upload attempt per call (honoring cooldowns) and is driven from the loop.

#pragma once
#include <Arduino.h>
#include <functional>
#include <vector>

enum class UploadResult : uint8_t {
  Success,      // accepted -> write sidecar
  AlreadyDone,  // duplicate / 409 -> treat as success
  Throttled,    // rate-limited / connection reset -> back off this service
  HardError,    // auth/malformed -> surface, don't hammer
  Transient     // connect/IO blip -> short retry
};

struct UploadService {
  const char* name;          // "wigle"
  const char* sidecarTag;    // "wigle" -> /file.log.wigle
  uint32_t    minIntervalMs; // floor between attempts to THIS service
  uint32_t    maxBackoffMs;  // cap on exponential backoff
  // Perform one upload of `path` to this service. Owns connect/framing/response
  // parsing for now; returns a classified result. (Wraps existing code in step 1.)
  std::function<UploadResult(const String& path)> doUpload;
};

class UploadManager {
public:
  void addService(const UploadService& svc);

  // Callbacks the manager uses without knowing SD/sidecar internals.
  void setScan(std::function<void(std::vector<String>&)> listLogs,
               std::function<bool(const String&, const char*)> sidecarExists,
               std::function<void(const String&, const char*)> writeSidecar);

  void begin();                 // build work list from listLogs + sidecar state
  bool idle(uint32_t now) const;// nothing left that is due to run

  // One paced step. Returns true if it attempted an upload this call.
  bool runNext(uint32_t now);

  struct Stats { int ok = 0, failed = 0, skipped = 0, pending = 0; };
  Stats stats() const { return stats_; }

  // Last thing that happened, for the display layer (honest state).
  const char* lastFile() const { return lastFile_.c_str(); }
  const char* lastService() const { return lastService_; }
  const char* lastStage() const { return lastStage_; }

private:
  struct Job { String path; uint8_t owed; };          // bitmask of services still owed
  struct SvcState { uint32_t nextAllowedMs = 0; uint32_t backoffMs = 0; };

  std::vector<UploadService> services_;
  std::vector<Job>           jobs_;
  std::vector<SvcState>      svc_;
  Stats stats_;
  String lastFile_;
  const char* lastService_ = "";
  const char* lastStage_ = "idle";

  std::function<void(std::vector<String>&)>            listLogs_;
  std::function<bool(const String&, const char*)>      sidecarExists_;
  std::function<void(const String&, const char*)>      writeSidecar_;
};
