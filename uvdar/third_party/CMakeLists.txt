set(KOMPUTE_OPT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(KOMPUTE_OPT_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(KOMPUTE_OPT_LOG_LEVEL Off CACHE STRING "" FORCE)
set(KOMPUTE_OPT_INSTALL OFF CACHE BOOL "" FORCE)
set(KOMPUTE_OPT_USE_BUILT_IN_FMT OFF CACHE BOOL "" FORCE)
set(KOMPUTE_OPT_USE_BUILT_IN_VULKAN_HEADER OFF CACHE BOOL "" FORCE)
set(KOMPUTE_OPT_DISABLE_VULKAN_VERSION_CHECK ON CACHE BOOL "" FORCE)

# Build kompute as a static library so it gets baked into Uvdar_UvLedDetectorLib.
# This avoids runtime linker failures: the shared libkompute.so is not installed
# (KOMPUTE_OPT_INSTALL OFF), so the installed Uvdar lib can't find it on
# incremental rebuilds when CMake strips the build-tree RPATH.
set(_saved_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(kompute)
set(BUILD_SHARED_LIBS ${_saved_BUILD_SHARED_LIBS})

