#include <uvdar_core/uv_led_detector/gpu_context.h>

namespace uvdar {

/* GetInstance //{ */
GpuContext& GpuContext::GetInstance() {
  static GpuContext instance;
  return instance;
}
//}

/* GpuContext //{ */
GpuContext::GpuContext() {
  gpu_idx_ = gpu_index_cached_();

  jobs_tokens_ = computeFamilySequence_();
  if (jobs_tokens_.empty()) {
    mgr_ = std::make_shared<kp::Manager>(gpu_idx_);

  } else {
    mgr_ = std::make_shared<kp::Manager>(gpu_idx_, jobs_tokens_);
  }
}
//}

/* gpu_idx //{ */
uint32_t GpuContext::gpu_idx() const {
  return gpu_idx_;
}
//}

/* manager //{ */
kp::Manager& GpuContext::manager() {
  return *mgr_;
}
//}

/* mutex //{ */
std::mutex& GpuContext::mutex() {
  return mtx_;
}
//}

/* getNextFreeFamilyIdx //{ */
uint32_t GpuContext::getNextFreeFamilyIdx() {
  if (jobs_tokens_.empty()) {
    return 0u;
  }

  int next_idx = jobs_tokens_.at(next_token_++);
  if (next_token_ >= jobs_tokens_.size()) {
    next_token_ = 0;
  }

  return next_idx;
}
//}

/* computeFamilySequence_ //{ */
std::vector<uint32_t> GpuContext::computeFamilySequence_() {

  kp::Manager mgr;
  if (gpu_idx_ == std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("GPU index has not been set. Cannot continue...");
  }

  auto scan_success = scanComputeFamilies(mgr.getVkInstance()->enumeratePhysicalDevices().at(gpu_idx_));
  if (!scan_success) {
    return {};
  }

  std::vector<uint32_t> tokens;
  auto gpu_families = compute_families_;
  size_t idx{0};
  while (!gpu_families.empty()) {
    if (idx >= gpu_families.size()) {
      idx = 0;
    }

    auto& family = gpu_families[idx];
    if (family.queue_count > 0) {
      tokens.push_back(family.id);
      family.queue_count--;
      idx++;
    } else {
      gpu_families.erase(gpu_families.begin() + idx);
    }
  }

  return tokens;
}
//}

/* gpu_index_cached_ //{ */
uint32_t GpuContext::gpu_index_cached_() {
  static const uint32_t idx = pick_discrete_else_integrated_gpu_();
  return idx;
}
//}

/* scanComputeFamilies //{ */
bool GpuContext::scanComputeFamilies(const vk::PhysicalDevice& device) {
  if (!compute_families_.empty()) {
    throw std::runtime_error("GPU: compute queue scanning has already happened.");
  }

  const std::vector<vk::QueueFamilyProperties> families = device.getQueueFamilyProperties();

  for (uint32_t familyIndex = 0; familyIndex < families.size(); ++familyIndex) {
    const auto& family    = families[familyIndex];
    const bool hasCompute = static_cast<bool>(family.queueFlags & vk::QueueFlagBits::eCompute);

    if (!hasCompute) {
      continue;
    }

    GpuComputeFamily f;
    f.id          = familyIndex;
    f.queue_count = family.queueCount;

    compute_families_.push_front(f);
  }

  if (compute_families_.empty()) {
    return false;
  }

  return true;
}
//}

/* pick_discrete_else_integrated_gpu //{ */
uint32_t GpuContext::pick_discrete_else_integrated_gpu_() {
  VkApplicationInfo app{};
  app.sType      = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.apiVersion = VK_API_VERSION_1_1;

  VkInstanceCreateInfo ci{};
  ci.sType            = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  ci.pApplicationInfo = &app;

  VkInstance instance{};
  if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create Vulkan instance");
  }

  uint32_t count = 0;
  VkResult r     = vkEnumeratePhysicalDevices(instance, &count, nullptr);
  if (r != VK_SUCCESS || count == 0) {
    vkDestroyInstance(instance, nullptr);
    throw std::runtime_error("No Vulkan physical devices found");
  }

  std::vector<VkPhysicalDevice> devices(count);
  r = vkEnumeratePhysicalDevices(instance, &count, devices.data());
  if (r != VK_SUCCESS) {
    vkDestroyInstance(instance, nullptr);
    throw std::runtime_error("Failed to enumerate Vulkan physical devices");
  }

  // Prefer discrete
  for (uint32_t i = 0; i < count; ++i) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(devices[i], &props);
    if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
      vkDestroyInstance(instance, nullptr);
      return i;
    }
  }

  // Then integrated
  for (uint32_t i = 0; i < count; ++i) {
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(devices[i], &props);
    if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
      vkDestroyInstance(instance, nullptr);
      return i;
    }
  }

  vkDestroyInstance(instance, nullptr);
  return 0;
}
//}

} // namespace uvdar