#pragma once

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace uvdar_core::app {

/**
 * @brief Resolve a TF frame name from a config file into this robot's
 * namespace.
 *
 * Follows the same convention ROS itself uses for topic/frame resolution: a
 * name with a leading '/' is GLOBAL and is returned unchanged, apart from
 * stripping the slash itself (tf2 frame ids never carry one); anything else
 * is RELATIVE and gets "<UAV_NAME>/" prepended automatically, e.g. "fcu" ->
 * "uav1/fcu" when UAV_NAME=uav1.
 *
 * Why this exists at all: fields like camera_frame/output_frame are plain
 * strings read by our own YAML loader, not ROS topics -- ROS2 launch's
 * namespace push (the `namespace=` argument, which handles topics/services/
 * actions) never touches them. Without this, a config has to hardcode
 * "uav1/bluefox_left" and silently breaks (wrong TF, no error) the moment
 * UAV_NAME differs, even though every topic in the same launch is correctly
 * namespaced. This is the config-side equivalent of ROS1 roslaunch's
 * $(arg uav_name) substitution.
 *
 * A config author writes relative names ("fcu", "bluefox_left") to make a
 * config portable across any UAV_NAME, or a leading-slash name ("/local_origin")
 * for a literal, deployment-independent frame that should never be
 * namespaced (e.g. a generic single-robot example config with no UAV_NAME at
 * all).
 */
inline std::string resolveFrameName(const std::string& value)
{
    if (value.empty() || value.front() == '/') {
        return value.empty() ? value : value.substr(1);
    }

    const char* uav_name = std::getenv("UAV_NAME");
    if (uav_name == nullptr || std::string(uav_name).empty()) {
        throw std::runtime_error(
            "Config value '" + value + "' is a relative TF frame name (no leading '/'), "
            "so it needs the UAV_NAME environment variable to resolve it against, but "
            "UAV_NAME is not set. Prefix the value with '/' if it should NOT be "
            "namespaced.");
    }

    return std::string(uav_name) + "/" + value;
}

} // namespace uvdar_core::app
