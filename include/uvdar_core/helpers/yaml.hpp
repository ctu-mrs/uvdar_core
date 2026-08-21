#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace uvdar_core::helpers::yaml {

/**
 * @brief Expand environment-variable references in a configuration string.
 *
 * Both $VARIABLE and ${VARIABLE} forms are supported. A referenced variable
 * must exist in the process environment; this prevents a silently incomplete
 * TF frame, topic, or file path from being used at runtime.
 */
inline std::string expandEnvironmentVariables(const std::string_view value)
{
    const auto is_name_start = [](const char character) {
        const unsigned char unsigned_character = static_cast<unsigned char>(character);
        return character == '_' || std::isalpha(unsigned_character) != 0;
    };
    const auto is_name_character = [&is_name_start](const char character) {
        const unsigned char unsigned_character = static_cast<unsigned char>(character);
        return is_name_start(character) || std::isdigit(unsigned_character) != 0;
    };

    std::string expanded;
    expanded.reserve(value.size());

    for (std::size_t index = 0; index < value.size();) {
        if (value[index] != '$') {
            expanded += value[index++];
            continue;
        }

        if (index + 1 >= value.size() || (value[index + 1] != '{' && !is_name_start(value[index + 1]))) {
            expanded += value[index++];
            continue;
        }

        std::string name;
        if (value[index + 1] == '{') {
            const std::size_t name_start = index + 2;
            const std::size_t name_end = value.find('}', name_start);
            if (name_end == std::string_view::npos) {
                throw std::runtime_error("Malformed environment-variable reference in YAML value '" + std::string(value) + "'.");
            }
            name.assign(value.substr(name_start, name_end - name_start));
            if (name.empty() || !is_name_start(name.front())
                || !std::all_of(name.begin() + 1, name.end(), is_name_character)) {
                throw std::runtime_error("Invalid environment-variable name '" + name + "' in YAML value '" + std::string(value) + "'.");
            }
            index = name_end + 1;
        } else {
            const std::size_t name_start = index + 1;
            std::size_t name_end = name_start + 1;
            while (name_end < value.size() && is_name_character(value[name_end])) {
                ++name_end;
            }
            name.assign(value.substr(name_start, name_end - name_start));
            index = name_end;
        }

        const char* const environment_value = std::getenv(name.c_str());
        if (environment_value == nullptr) {
            throw std::runtime_error("Environment variable '" + name + "' referenced by YAML configuration is not set.");
        }
        expanded += environment_value;
    }

    return expanded;
}

/**
 * @brief Expand environment-variable references in every YAML scalar value.
 *
 * Mapping keys are intentionally left unchanged; variables are configuration
 * values, not schema names.
 */
inline void expandEnvironmentVariables(YAML::Node node)
{
    if (!node) {
        return;
    }

    if (node.IsScalar()) {
        node = expandEnvironmentVariables(node.as<std::string>());
        return;
    }

    if (node.IsSequence()) {
        for (const YAML::Node& child : node) {
            expandEnvironmentVariables(child);
        }
        return;
    }

    if (node.IsMap()) {
        for (auto iterator = node.begin(); iterator != node.end(); ++iterator) {
            expandEnvironmentVariables(iterator->second);
        }
    }
}

/**
 * @brief Load a YAML file and expand $VARIABLE and ${VARIABLE} values.
 */
inline YAML::Node loadFile(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);
    expandEnvironmentVariables(root);
    return root;
}

} // namespace uvdar_core::helpers::yaml
