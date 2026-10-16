/**
 * @file sg100_angle_map.h
 * @brief SG100 joint angle map configuration and validation API.
 */
#ifndef SG100_ANGLE_MAP_H
#define SG100_ANGLE_MAP_H

#include "sg100_types.h"
#include <array>
#include <cstdint>
#include <string>

#ifdef _WIN32
    #ifdef SG100_SDK_EXPORTS
        #define SG100_ANGLE_MAP_API __declspec(dllexport)
    #else
        #define SG100_ANGLE_MAP_API __declspec(dllimport)
    #endif
#else
    #define SG100_ANGLE_MAP_API
#endif

enum class SG100AngleMapHand : uint8_t {
    RIGHT = 0,
    LEFT = 1
};

enum class SG100AngleMapSource : uint8_t {
    NONE = 0,
    V3 = 1,
    LEGACY = 2,
    BUILTIN_DEFAULT = 3
};

enum class SG100AngleMapError : uint8_t {
    NONE = 0,
    FILE_NOT_FOUND,
    FILE_OPEN_ERROR,
    JSON_PARSE_ERROR,
    ROOT_NOT_OBJECT,
    UNSUPPORTED_VERSION,
    MISSING_JOINT_ANGLE_MAP,
    INVALID_JOINT_ANGLE_MAP,
    MISSING_HAND_MAP,
    INVALID_HAND_MAP,
    INVALID_JOINT_COUNT,
    INVALID_JOINT_ENTRY,
    MISSING_FIELD,
    INVALID_NUMBER,
    INVALID_SCALE,
    INVALID_RANGE
};

struct SG100AngleMapParam {
    float a;
    float b;
    float min_angle;
    float max_angle;
};

using SG100AngleMapParams = std::array<SG100AngleMapParam, JOINT_COUNT>;

struct SG100_ANGLE_MAP_API SG100AngleMapStatus {
    bool success;
    SG100AngleMapSource source;
    SG100AngleMapError error;
    SG100AngleMapHand hand;
    uint8_t joint_count;
    std::string schema_version;
    std::string source_file;
    std::string error_message;

    SG100AngleMapStatus();
};

/**
 * @brief Load one angle map file.
 *
 * The v3 schema requires version "3.0", both L/R arrays, and exactly
 * JOINT_COUNT validated entries. Legacy files remain supported and ignore
 * the hand argument because they contain one flat map.
 */
SG100_ANGLE_MAP_API SG100AngleMapStatus LoadJointAngleMapFile(
    const std::string& path,
    SG100AngleMapHand hand,
    SG100AngleMapParams& out);

/**
 * @brief Load the preferred map from a configuration directory.
 *
 * joint_angle_map_v3.0.json has priority over joint_angle_map.json. If the
 * preferred v3 file exists but is invalid, this function fails and does not
 * silently fall back to the legacy file.
 */
SG100_ANGLE_MAP_API SG100AngleMapStatus LoadJointAngleMapFromSearchDir(
    const std::string& config_search_dir,
    SG100AngleMapHand hand,
    SG100AngleMapParams& out);

/** @brief Construct the SDK's built-in compatibility defaults. */
SG100_ANGLE_MAP_API SG100AngleMapParams BuildDefaultJointAngleMap();

#endif // SG100_ANGLE_MAP_H
