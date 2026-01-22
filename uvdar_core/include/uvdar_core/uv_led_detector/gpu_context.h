#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>

#include <kompute/Kompute.hpp>
#include <vulkan/vulkan.h>

namespace uvdar {

/* GpuContext //{ */
class GpuContext {
 public:
  GpuContext(const GpuContext&)            = delete; // cannot be cloneable
  GpuContext& operator=(const GpuContext&) = delete; // cannot be assignable

  static GpuContext& GetInstance();

  uint32_t gpu_idx() const;
  kp::Manager& manager();
  std::mutex& mutex();

 private:
  GpuContext();

  uint32_t gpu_index_cached_();
  uint32_t pick_discrete_else_integrated_gpu_();

 private:
  uint32_t gpu_idx_;
  std::shared_ptr<kp::Manager> mgr_;
  std::mutex mtx_;
};
//}

} // namespace uvdar