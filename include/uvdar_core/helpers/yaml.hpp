#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

/**
 * @brief Read an optional YAML scalar, returning @p fallback when absent.
 */
template <typename T>
inline T optionalScalar(const YAML::Node& node, const std::string& key, T fallback)
{
    const YAML::Node value = node[key];
    return value ? value.as<T>() : fallback;
}

/**
 * @brief Read a required YAML scalar and identify its containing section on error.
 */
template <typename T>
inline T requireScalar(const YAML::Node& node, const std::string& key, const std::string& section = "configuration")
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error("Missing required " + section + " key '" + key + "'.");
    }
    return value.as<T>();
}

/**
 * @brief Read a required YAML mapping or sequence node.
 */
inline YAML::Node requireNode(const YAML::Node& node, const std::string& key, const std::string& section = "configuration")
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error("Missing required " + section + " section '" + key + "'.");
    }
    return value;
}

/**
 * @brief Prefer a scalar from @p primary and otherwise read it from @p fallback.
 */
template <typename T>
inline T optionalScalarAny(
    const YAML::Node& primary,
    const YAML::Node& fallback,
    const std::string& key,
    T default_value)
{
    if (primary && primary[key]) {
        return primary[key].as<T>();
    }
    return optionalScalar<T>(fallback, key, default_value);
}

/**
 * @brief Read an optional sequence of scalars; return an empty vector otherwise.
 */
template <typename T>
inline std::vector<T> optionalSequence(const YAML::Node& node, const std::string& key)
{
    const YAML::Node sequence = node[key];
    if (!sequence || !sequence.IsSequence()) {
        return {};
    }

    std::vector<T> values;
    values.reserve(sequence.size());
    for (const YAML::Node& value : sequence) {
        values.push_back(value.as<T>());
    }
    return values;
}

/**
 * @brief Prefer a scalar sequence from @p primary and otherwise use @p fallback.
 */
template <typename T>
inline std::vector<T> optionalSequenceAny(const YAML::Node& primary, const YAML::Node& fallback, const std::string& key)
{
    std::vector<T> values = optionalSequence<T>(primary, key);
    return values.empty() ? optionalSequence<T>(fallback, key) : values;
}

/**
 * @brief Resolve a non-empty configuration path relative to its containing file.
 */
inline std::string resolvePath(const std::filesystem::path& config_path, const std::string& value)
{
    if (value.empty()) {
        return {};
    }

    const std::filesystem::path path(value);
    if (path.is_absolute()) {
        return path.string();
    }
    return (config_path.parent_path() / path).lexically_normal().string();
}

} // namespace uvdar_core::helpers::yaml
