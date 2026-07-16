#include "UploadManager.h"

void UploadManager::addService(const UploadService& svc) {
  services_.push_back(svc);
  svc_.push_back(SvcState{});
}

void UploadManager::setScan(std::function<void(std::vector<String>&)> listLogs,
                            std::function<bool(const String&, const char*)> sidecarExists,
                            std::function<void(const String&, const char*)> writeSidecar) {
  listLogs_ = listLogs;
  sidecarExists_ = sidecarExists;
  writeSidecar_ = writeSidecar;
}

void UploadManager::begin() {
  jobs_.clear();
  stats_ = Stats{};
  std::vector<String> logs;
  if (listLogs_) listLogs_(logs);

  for (auto& path : logs) {
    uint8_t owed = 0;
    for (size_t i = 0; i < services_.size(); i++) {
      if (!sidecarExists_ || !sidecarExists_(path, services_[i].sidecarTag))
        owed |= (1 << i);
    }
    if (owed) jobs_.push_back(Job{path, owed});
    else stats_.skipped++;
  }
  stats_.pending = (int)jobs_.size();
}

bool UploadManager::idle(uint32_t now) const {
  for (const auto& job : jobs_) {
    if (!job.owed) continue;
    for (size_t i = 0; i < services_.size(); i++) {
      if (!(job.owed & (1 << i))) continue;
      if ((int32_t)(now - svc_[i].nextAllowedMs) >= 0) return false; // something due
    }
  }
  return true;
}

bool UploadManager::runNext(uint32_t now) {
  // Find the first job/service pair that is owed AND whose service cooldown is up.
  for (auto& job : jobs_) {
    if (!job.owed) continue;
    for (size_t i = 0; i < services_.size(); i++) {
      uint8_t bit = (1 << i);
      if (!(job.owed & bit)) continue;
      if ((int32_t)(now - svc_[i].nextAllowedMs) < 0) continue; // service backing off

      const UploadService& svc = services_[i];
      lastFile_ = job.path;
      lastService_ = svc.name;
      lastStage_ = "uploading";

      UploadResult r = svc.doUpload ? svc.doUpload(job.path) : UploadResult::HardError;

      switch (r) {
        case UploadResult::Success:
        case UploadResult::AlreadyDone:
          if (writeSidecar_) writeSidecar_(job.path, svc.sidecarTag);
          job.owed &= ~bit;
          svc_[i].backoffMs = 0;
          svc_[i].nextAllowedMs = now + svc.minIntervalMs; // pace even on success
          stats_.ok++;
          lastStage_ = "ok";
          break;

        case UploadResult::Throttled:
        case UploadResult::Transient: {
          uint32_t b = svc_[i].backoffMs ? svc_[i].backoffMs * 2
                                         : (svc.minIntervalMs ? svc.minIntervalMs : 1000);
          if (b > svc.maxBackoffMs) b = svc.maxBackoffMs;
          svc_[i].backoffMs = b;
          svc_[i].nextAllowedMs = now + b;               // don't hammer this service
          stats_.failed++;
          lastStage_ = "backoff";
          break;
        }

        case UploadResult::HardError:
        default:
          // Give up on THIS service for THIS file this session; surface it.
          job.owed &= ~bit;
          svc_[i].nextAllowedMs = now + svc.minIntervalMs;
          stats_.failed++;
          lastStage_ = "error";
          break;
      }

      // Recompute pending (jobs with any owed bit left).
      int pending = 0;
      for (auto& j : jobs_) if (j.owed) pending++;
      stats_.pending = pending;
      return true; // one attempt per call
    }
  }
  return false; // nothing due right now
}
