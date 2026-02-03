#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <optional>
#include <limits>
#include <deque>
#include <vector>

namespace kp {
class Manager;
}

namespace vk {
class PhysicalDevice;
}

namespace uvdar {

struct GpuComputeFamily {
  uint32_t id;
  uint32_t queue_count;
};

/* GpuContext //{ */
class GpuContext {
 public:
  GpuContext(const GpuContext&)            = delete; // cannot be cloneable
  GpuContext& operator=(const GpuContext&) = delete; // cannot be assignable

  static GpuContext& GetInstance();

  uint32_t gpu_idx() const;
  kp::Manager& manager();
  std::mutex& mutex();

  uint32_t getNextFreeFamilyIdx();
  bool scanComputeFamilies(const vk::PhysicalDevice& device);
  std::vector<uint32_t> computeFamilySequence_();

 private:
  GpuContext();

  uint32_t getGpuIdx_();
  uint32_t pickDiscreteElseIntegratedGpu_();

 private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
  std::mutex mtx_;
};
//}

} // namespace uvdar