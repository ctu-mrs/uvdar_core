#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <optional>
#include <limits>
#include <deque>

#include <kompute/Kompute.hpp>
#include <vulkan/vulkan.h>

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

  uint32_t gpu_index_cached_();
  uint32_t pick_discrete_else_integrated_gpu_();

 private:
  uint32_t gpu_idx_{std::numeric_limits<uint32_t>::max()};
  std::shared_ptr<kp::Manager> mgr_;
  std::mutex mtx_;

  std::deque<GpuComputeFamily> compute_families_;
  std::vector<uint32_t> jobs_tokens_;
  uint32_t next_token_{0u};

  bool initialized{false};
};
//}

} // namespace uvdar