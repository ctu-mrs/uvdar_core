#include <uvdar/uv_led_detector/gpu_context.h>
#include <vulkan/vulkan.h>
#include <kompute/Kompute.hpp>

namespace uvdar {

/* GpuContext::Impl //{ */
struct GpuContext::Impl {
  uint32_t gpu_idx{std::numeric_limits<uint32_t>::max()};

  std::shared_ptr<kp::Manager> mgr;

  std::deque<GpuComputeFamily> compute_families;
  std::vector<uint32_t> jobs_tokens;
  uint32_t next_token{0u};

  bool initialized{false};
};
//}

/* GetInstance //{ */
GpuContext& GpuContext::GetInstance() {
  static GpuContext instance;
  return instance;
}
//}

/* GpuContext //{ */
GpuContext::GpuContext() : pimpl_(std::make_unique<Impl>()) {
  pimpl_->gpu_idx = getGpuIdx_();

  pimpl_->jobs_tokens = computeFamilySequence_();
  if (pimpl_->jobs_tokens.empty()) {
    pimpl_->mgr = std::make_shared<kp::Manager>(pimpl_->gpu_idx);

  } else {
    pimpl_->mgr = std::make_shared<kp::Manager>(pimpl_->gpu_idx, pimpl_->jobs_tokens);
  }
}
//}

/* gpu_idx //{ */
uint32_t GpuContext::gpu_idx() const {
  return pimpl_->gpu_idx;
}
//}

/* manager //{ */
kp::Manager& GpuContext::manager() {
  return *pimpl_->mgr;
}
//}

/* mutex //{ */
std::mutex& GpuContext::mutex() {
  return mtx_;
}
//}

/* getNextFreeFamilyIdx //{ */
uint32_t GpuContext::getNextFreeFamilyIdx() {
  if (pimpl_->jobs_tokens.empty()) {
    return 0u;
  }

  int next_idx = pimpl_->jobs_tokens.at(pimpl_->next_token++);
  if (pimpl_->next_token >= pimpl_->jobs_tokens.size()) {
    pimpl_->next_token = 0;
  }

  return next_idx;
}
//}

/* computeFamilySequence_ //{ */
std::vector<uint32_t> GpuContext::computeFamilySequence_() {

  kp::Manager mgr;
  if (pimpl_->gpu_idx == std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error("GPU index has not been set. Cannot continue...");
  }

  auto scan_success = scanComputeFamilies(mgr.getVkInstance()->enumeratePhysicalDevices().at(pimpl_->gpu_idx));
  if (!scan_success) {
    return {};
  }

  std::vector<uint32_t> tokens;
  auto gpu_families = pimpl_->compute_families;
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

/* getGpuIdx_ //{ */
uint32_t GpuContext::getGpuIdx_() {
  static const uint32_t idx = pickDiscreteElseIntegratedGpu_();
  return idx;
}
//}

/* scanComputeFamilies //{ */
bool GpuContext::scanComputeFamilies(const vk::PhysicalDevice& device) {
  if (!pimpl_->compute_families.empty()) {
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

    pimpl_->compute_families.push_front(f);
  }

  if (pimpl_->compute_families.empty()) {
    return false;
  }

  return true;
}
//}

/* pickDiscreteElseIntegratedGpu_ //{ */
uint32_t GpuContext::pickDiscreteElseIntegratedGpu_() {
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