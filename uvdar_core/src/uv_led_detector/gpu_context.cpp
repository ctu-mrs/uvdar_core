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
  mgr_     = std::make_shared<kp::Manager>(gpu_idx_);
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

/* gpu_index_cached_ //{ */
uint32_t GpuContext::gpu_index_cached_() {
  static const uint32_t idx = pick_discrete_else_integrated_gpu_();
  return idx;
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