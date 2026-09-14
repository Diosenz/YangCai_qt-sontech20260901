/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsTextConfigReader.cpp
 * @brief     IBVS 文本配置读取实现。
 * @details   本文件解析 IBVS 运行默认值和工件 profile 的 INI-like txt 文件。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 15 日
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

#include <cctype>

#include "ibvs/IbvsTextConfigReader.h"

namespace i2r_rlc::ibvs {

namespace {

using KeyValueList = std::vector<std::pair<std::string, std::string>>;

struct TextSection
{
    std::string name;
    KeyValueList values;
};

struct TextDocument
{
    IbvsError error;
    std::vector<TextSection> sections;
};

IbvsError makeOpenError(const std::string& path)
{
    return {
        IbvsStatus::FILE_OPEN_FAILED,
        "failed to open config file: " + path
    };
}

std::string trim(const std::string& value)
{
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
        return std::string();
    }

    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string toLower(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
    return value;
}

std::string stripInlineComment(const std::string& line)
{
    const std::size_t comment = line.find('#');
    if (comment == std::string::npos)
    {
        return line;
    }

    return line.substr(0, comment);
}

std::ifstream openInputFile(const std::string& path)
{
    return std::ifstream(std::filesystem::u8path(path));
}

TextDocument readDocument(const std::string& path)
{
    TextDocument document;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        document.error = makeOpenError(path);
        return document;
    }

    TextSection* currentSection = NULL;
    std::string line;
    while (std::getline(file, line))
    {
        const std::string content = trim(stripInlineComment(line));
        if (content.empty())
        {
            continue;
        }

        if (content.front() == '[' && content.back() == ']')
        {
            TextSection section;
            section.name = trim(content.substr(1, content.size() - 2));
            document.sections.push_back(section);
            currentSection = &document.sections.back();
            continue;
        }

        const std::size_t equal = content.find('=');
        if (equal == std::string::npos || currentSection == NULL)
        {
            document.error = {
                IbvsStatus::PARSE_ERROR,
                "invalid config line: " + content
            };
            return document;
        }

        currentSection->values.push_back({
            toLower(trim(content.substr(0, equal))),
            trim(content.substr(equal + 1))
        });
    }

    document.error = { IbvsStatus::OK, std::string() };
    return document;
}

const TextSection* findSection(
    const TextDocument& document,
    const std::string& sectionName)
{
    const std::string wanted = toLower(sectionName);
    for (const TextSection& section : document.sections)
    {
        if (toLower(section.name) == wanted)
        {
            return &section;
        }
    }

    return NULL;
}

std::string getValue(
    const TextSection& section,
    const std::string& key,
    const std::string& fallback)
{
    const std::string wanted = toLower(key);
    for (const auto& entry : section.values)
    {
        if (entry.first == wanted)
        {
            return entry.second;
        }
    }

    return fallback;
}

bool hasValue(const TextSection& section, const std::string& key)
{
    const std::string wanted = toLower(key);
    for (const auto& entry : section.values)
    {
        if (entry.first == wanted)
        {
            return true;
        }
    }

    return false;
}

std::vector<double> parseDoubleList(const std::string& value)
{
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), ';', ' ');

    std::istringstream stream(normalized);
    std::vector<double> values;
    double item = 0.0;
    while (stream >> item)
    {
        values.push_back(item);
    }
    return values;
}

std::vector<int> parseIntList(const std::string& value)
{
    std::string normalized = value;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), ';', ' ');

    std::istringstream stream(normalized);
    std::vector<int> values;
    int item = 0;
    while (stream >> item)
    {
        values.push_back(item);
    }
    return values;
}

double readDouble(
    const TextSection& section,
    const std::string& key,
    double fallback)
{
    if (!hasValue(section, key))
    {
        return fallback;
    }

    const std::vector<double> values = parseDoubleList(
        getValue(section, key, std::string()));
    if (values.empty() || !std::isfinite(values.front()))
    {
        return fallback;
    }

    return values.front();
}

int readInt(const TextSection& section, const std::string& key, int fallback)
{
    if (!hasValue(section, key))
    {
        return fallback;
    }

    const std::vector<int> values = parseIntList(
        getValue(section, key, std::string()));
    return values.empty() ? fallback : values.front();
}

bool readBool(const TextSection& section, const std::string& key, bool fallback)
{
    if (!hasValue(section, key))
    {
        return fallback;
    }

    const std::string value = toLower(getValue(section, key, std::string()));
    if (value == "1" || value == "true" || value == "yes" || value == "on")
    {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off")
    {
        return false;
    }

    return fallback;
}

void readFeatureGeometry(
    const TextSection& section,
    IbvsFeatureGeometryParameters* parameters)
{
    parameters->h1Mm = readDouble(section, "h1_mm", parameters->h1Mm);
    parameters->h2Mm = readDouble(section, "h2_mm", parameters->h2Mm);
    parameters->focalLengthPx = readDouble(
        section,
        "focal_length_px",
        parameters->focalLengthPx);
    parameters->torchTiltScale = readDouble(
        section,
        "torch_tilt_scale",
        parameters->torchTiltScale);
    parameters->useDepthBasedH2 = readBool(
        section,
        "use_depth_based_h2",
        parameters->useDepthBasedH2);
    parameters->minFeatureDistancePx = readDouble(
        section,
        "min_feature_distance_px",
        parameters->minFeatureDistancePx);
    parameters->useOverrideH2Px = readBool(
        section,
        "use_override_h2_px",
        parameters->useOverrideH2Px);
    parameters->overrideH2Px = readDouble(
        section,
        "override_h2_px",
        parameters->overrideH2Px);
    parameters->useOverrideUnitDirection = readBool(
        section,
        "use_override_unit_direction",
        parameters->useOverrideUnitDirection);

    const std::vector<double> direction = parseDoubleList(
        getValue(section, "override_unit_direction", std::string()));
    if (direction.size() >= 2)
    {
        parameters->overrideUnitDirection = { direction[0], direction[1] };
    }
    else
    {
        parameters->overrideUnitDirection.u = readDouble(
            section,
            "override_unit_u",
            parameters->overrideUnitDirection.u);
        parameters->overrideUnitDirection.v = readDouble(
            section,
            "override_unit_v",
            parameters->overrideUnitDirection.v);
    }
}

void readFusion(
    const TextSection& section,
    IbvsFusionConfig* config)
{
    config->minTorchConfidence = readDouble(
        section,
        "min_torch_confidence",
        config->minTorchConfidence);
    config->minLvsConfidence = readDouble(
        section,
        "min_lvs_confidence",
        config->minLvsConfidence);
    config->minLvsCenterlineQuality = readDouble(
        section,
        "min_lvs_centerline_quality",
        config->minLvsCenterlineQuality);
    config->maxFrameDelta = readInt(
        section,
        "max_frame_delta",
        config->maxFrameDelta);
    config->preferLvs = readBool(section, "prefer_lvs", config->preferLvs);
    config->lvsChannelSigmaScale = readDouble(
        section,
        "lvs_channel_sigma_scale",
        config->lvsChannelSigmaScale);
    config->fallbackChannelSigmaScale = readDouble(
        section,
        "fallback_channel_sigma_scale",
        config->fallbackChannelSigmaScale);

    const std::vector<double> sigmas = parseDoubleList(
        getValue(section, "base_channel_sigmas_px", std::string()));
    if (sigmas.size() >= 4)
    {
        config->baseChannelSigmasPx << sigmas[0], sigmas[1], sigmas[2], sigmas[3];
    }
}

void readUnifiedVelocity(
    const TextSection& section,
    IbvsUnifiedVelocityConfig* config)
{
    config->lambdaF = readDouble(section, "lambda_f", config->lambdaF);
    config->dampingKappa = readDouble(
        section,
        "damping_kappa",
        config->dampingKappa);
    config->minDamping = readDouble(
        section,
        "min_damping",
        config->minDamping);
    config->maxVelocityMps = readDouble(
        section,
        "max_velocity_mps",
        config->maxVelocityMps);
    config->lvsTaskScaleMin = readDouble(
        section,
        "lvs_task_scale_min",
        config->lvsTaskScaleMin);
    config->lvsTaskScaleMax = readDouble(
        section,
        "lvs_task_scale_max",
        config->lvsTaskScaleMax);
}

void readLvsConstraint(
    const TextSection& section,
    IbvsLvsConstraint* constraint)
{
    constraint->lambdaD = readDouble(section, "lambda_d", constraint->lambdaD);
    constraint->alphaLvs = readDouble(
        section,
        "alpha_lvs",
        constraint->alphaLvs);
    constraint->omegaLvs = readDouble(
        section,
        "omega_lvs",
        constraint->omegaLvs);
    constraint->betaLvs = readDouble(
        section,
        "beta_lvs",
        constraint->betaLvs);
    constraint->useAxisWeights = readBool(
        section,
        "use_axis_weights",
        constraint->useAxisWeights);

    const std::vector<double> axisWeights = parseDoubleList(
        getValue(section, "axis_weights", std::string()));
    if (axisWeights.size() >= 3)
    {
        constraint->axisWeights << axisWeights[0], axisWeights[1], axisWeights[2];
    }
}

void readControl(const TextSection& section, IbvsControlParameters* control)
{
    control->lambda = readDouble(section, "lambda", control->lambda);
    control->stopThresholdPx = readDouble(
        section,
        "stop_threshold_px",
        control->stopThresholdPx);
    control->dampingKappa = readDouble(
        section,
        "damping_kappa",
        control->dampingKappa);
    control->minDamping = readDouble(section, "min_damping", control->minDamping);
    control->maxCameraVelocityMps = readDouble(
        section,
        "max_camera_velocity_mps",
        control->maxCameraVelocityMps);
    control->useChannelwiseR = readBool(
        section,
        "use_channelwise_r",
        control->useChannelwiseR);

    const std::vector<double> sigmas = parseDoubleList(
        getValue(section, "channel_sigmas_px", std::string()));
    if (sigmas.size() >= 4)
    {
        control->channelSigmasPx << sigmas[0], sigmas[1], sigmas[2], sigmas[3];
    }
}

void readKalman(const TextSection& section, IbvsKalmanConfig* config)
{
    config->q = readDouble(section, "q", config->q);
    config->rPix = readDouble(section, "r_pix", config->rPix);
    config->p0 = readDouble(section, "p0", config->p0);
    config->epsUpdateM = readDouble(section, "eps_update_m", config->epsUpdateM);
    config->nisRejectThreshold = readDouble(
        section,
        "nis_reject_threshold",
        config->nisRejectThreshold);
    config->nisInflateThreshold = readDouble(
        section,
        "nis_inflate_threshold",
        config->nisInflateThreshold);
    config->nisInflateFactor = readDouble(
        section,
        "nis_inflate_factor",
        config->nisInflateFactor);
    config->maxHistorySize = static_cast<std::size_t>(std::max(
        0,
        readInt(section, "max_history_size", static_cast<int>(config->maxHistorySize))));
    config->useChannelwiseR = readBool(
        section,
        "use_channelwise_r",
        config->useChannelwiseR);

    const std::vector<double> sigmas = parseDoubleList(
        getValue(section, "channel_sigmas_px", std::string()));
    if (sigmas.size() >= 4)
    {
        config->channelSigmasPx << sigmas[0], sigmas[1], sigmas[2], sigmas[3];
    }
}

bool isWorkpieceSection(const TextSection& section)
{
    const std::string name = toLower(section.name);
    return name.rfind("workpiece ", 0) == 0;
}

std::string workpieceIdFromSection(const TextSection& section)
{
    return trim(section.name.substr(std::string("workpiece ").size()));
}

void readWorkpieceGeometry(
    const TextSection& section,
    IbvsWorkpieceProfile* profile)
{
    if (hasValue(section, "feature_geometry_h1_mm")
        || hasValue(section, "feature_geometry_h2_mm")
        || hasValue(section, "feature_geometry_override_h2_px")
        || hasValue(section, "feature_geometry_override_unit_direction"))
    {
        profile->hasFeatureGeometry = true;
    }

    profile->featureGeometry.h1Mm = readDouble(
        section,
        "feature_geometry_h1_mm",
        profile->featureGeometry.h1Mm);
    profile->featureGeometry.h2Mm = readDouble(
        section,
        "feature_geometry_h2_mm",
        profile->featureGeometry.h2Mm);
    profile->featureGeometry.useOverrideH2Px = readBool(
        section,
        "feature_geometry_use_override_h2_px",
        profile->featureGeometry.useOverrideH2Px);
    profile->featureGeometry.overrideH2Px = readDouble(
        section,
        "feature_geometry_override_h2_px",
        profile->featureGeometry.overrideH2Px);
    profile->featureGeometry.useOverrideUnitDirection = readBool(
        section,
        "feature_geometry_use_override_unit_direction",
        profile->featureGeometry.useOverrideUnitDirection);

    const std::vector<double> direction = parseDoubleList(
        getValue(
            section,
            "feature_geometry_override_unit_direction",
            std::string()));
    if (direction.size() >= 2)
    {
        profile->featureGeometry.overrideUnitDirection = {
            direction[0],
            direction[1]
        };
    }
}

void readWorkpieceMetadata(
    const TextSection& section,
    IbvsWorkpieceProfile* profile)
{
    profile->displayName = getValue(section, "display_name", profile->displayName);
    profile->legacyName = getValue(section, "legacy_name", profile->legacyName);
    profile->shape = getValue(section, "shape", profile->shape);
    profile->robotSimModelId = getValue(
        section,
        "robotsim_model_id",
        profile->robotSimModelId);
    profile->robotSimModelPath = getValue(
        section,
        "robotsim_model_path",
        profile->robotSimModelPath);
    profile->calibrationFile = getValue(
        section,
        "calibration_file",
        profile->calibrationFile);
}

void readWorkpieceLocalization(
    const TextSection& section,
    IbvsWorkpieceProfile* profile)
{
    profile->localizationRoiExpandPx = readInt(
        section,
        "localization_roi_expand_px",
        profile->localizationRoiExpandPx);
    profile->localizationRoiTopScale = readDouble(
        section,
        "localization_roi_top_scale",
        profile->localizationRoiTopScale);
    profile->localizationRoiBottomScale = readDouble(
        section,
        "localization_roi_bottom_scale",
        profile->localizationRoiBottomScale);
    profile->localizationCenterlineThreshold = readInt(
        section,
        "localization_centerline_threshold",
        profile->localizationCenterlineThreshold);
    profile->localizationPointCloudStride = readInt(
        section,
        "localization_pointcloud_stride",
        profile->localizationPointCloudStride);
    profile->localizationSlidingWindowFrames = readInt(
        section,
        "localization_sliding_window_frames",
        profile->localizationSlidingWindowFrames);
    profile->localizationMaxPointCount = readInt(
        section,
        "localization_max_point_count",
        profile->localizationMaxPointCount);
    profile->localizationIcpDof = getValue(
        section,
        "localization_icp_dof",
        profile->localizationIcpDof);
}

void readWorkpieceLvsFusion(
    const TextSection& section,
    IbvsWorkpieceProfile* profile)
{
    if (hasValue(section, "lvs_min_confidence")
        || hasValue(section, "lvs_min_centerline_quality"))
    {
        profile->hasLvsFusionThresholds = true;
    }

    profile->minLvsConfidence = readDouble(
        section,
        "lvs_min_confidence",
        profile->minLvsConfidence);
    profile->minLvsCenterlineQuality = readDouble(
        section,
        "lvs_min_centerline_quality",
        profile->minLvsCenterlineQuality);
}

void readWorkpiecePosition(
    const TextSection& section,
    IbvsWorkpieceProfile* profile)
{
    const std::vector<double> minMax = parseDoubleList(
        getValue(section, "unified_position_min_max_mm", std::string()));
    const std::vector<double> center = parseDoubleList(
        getValue(section, "unified_position_center_mm", std::string()));
    const std::vector<double> tolerance = parseDoubleList(
        getValue(section, "unified_position_tolerance_mm", std::string()));

    if (minMax.size() >= 6)
    {
        profile->hasPositionConstraint = true;
        profile->positionConstraint.enabled = true;
        profile->positionConstraint.xMinMm = minMax[0];
        profile->positionConstraint.xMaxMm = minMax[1];
        profile->positionConstraint.yMinMm = minMax[2];
        profile->positionConstraint.yMaxMm = minMax[3];
        profile->positionConstraint.zMinMm = minMax[4];
        profile->positionConstraint.zMaxMm = minMax[5];
    }
    else if (center.size() >= 3 && tolerance.size() >= 3)
    {
        profile->hasPositionConstraint = true;
        profile->positionConstraint.enabled = true;
        profile->positionConstraint.xMinMm = center[0] - tolerance[0];
        profile->positionConstraint.xMaxMm = center[0] + tolerance[0];
        profile->positionConstraint.yMinMm = center[1] - tolerance[1];
        profile->positionConstraint.yMaxMm = center[1] + tolerance[1];
        profile->positionConstraint.zMinMm = center[2] - tolerance[2];
        profile->positionConstraint.zMaxMm = center[2] + tolerance[2];
    }

    profile->positionConstraint.lambdaPos = readDouble(
        section,
        "unified_position_lambda",
        profile->positionConstraint.lambdaPos);
    profile->positionConstraint.omegaPos = readDouble(
        section,
        "unified_position_omega",
        profile->positionConstraint.omegaPos);
}

} // namespace

IbvsRuntimeTextConfigResult IbvsTextConfigReader::readRuntimeDefaults(
    const std::string& path)
{
    IbvsRuntimeTextConfigResult result;
    const TextDocument document = readDocument(path);
    if (!document.error.isOk())
    {
        result.error = document.error;
        return result;
    }

    if (const TextSection* section = findSection(document, "feature_geometry"))
    {
        readFeatureGeometry(*section, &result.config.featureGeometry);
        result.config.fusion.geometryParameters = result.config.featureGeometry;
    }
    if (const TextSection* section = findSection(document, "unified_velocity"))
    {
        readUnifiedVelocity(*section, &result.config.unifiedVelocity);
    }
    if (const TextSection* section = findSection(document, "lvs_constraint"))
    {
        readLvsConstraint(*section, &result.config.lvsConstraint);
    }
    if (const TextSection* section = findSection(document, "sensor_fusion"))
    {
        readFusion(*section, &result.config.fusion);
        result.config.fusion.geometryParameters = result.config.featureGeometry;
    }
    if (const TextSection* section = findSection(document, "ibvs_control"))
    {
        readControl(*section, &result.config.control);
    }
    if (const TextSection* section = findSection(document, "ibvs_kalman"))
    {
        readKalman(*section, &result.config.kalman);
    }

    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

IbvsRuntimeTextConfigResult
IbvsTextConfigReader::readRuntimeDefaultsWithFallback(
    const std::string& primaryPath,
    const std::string& fallbackPath)
{
    IbvsRuntimeTextConfigResult result = readRuntimeDefaults(primaryPath);
    if (result.error.isOk())
    {
        return result;
    }

    return readRuntimeDefaults(fallbackPath);
}

IbvsWorkpieceProfilesResult IbvsTextConfigReader::readWorkpieceProfiles(
    const std::string& path)
{
    IbvsWorkpieceProfilesResult result;
    const TextDocument document = readDocument(path);
    if (!document.error.isOk())
    {
        result.error = document.error;
        return result;
    }

    if (const TextSection* section = findSection(document, "selection"))
    {
        result.defaultWorkpieceId = getValue(
            *section,
            "default_workpiece_id",
            std::string());
    }

    for (const TextSection& section : document.sections)
    {
        if (!isWorkpieceSection(section))
        {
            continue;
        }

        IbvsWorkpieceProfile profile;
        profile.id = getValue(section, "id", workpieceIdFromSection(section));
        profile.legacyTypeAliases = parseIntList(
            getValue(section, "legacy_type_aliases", std::string()));
        readWorkpieceMetadata(section, &profile);
        readWorkpieceLocalization(section, &profile);
        readWorkpieceGeometry(section, &profile);
        readWorkpieceLvsFusion(section, &profile);
        readWorkpiecePosition(section, &profile);
        result.profiles.push_back(profile);
    }

    result.error = result.profiles.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no workpiece profiles found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

IbvsWorkpieceProfilesResult
IbvsTextConfigReader::readWorkpieceProfilesWithFallback(
    const std::string& primaryPath,
    const std::string& fallbackPath)
{
    IbvsWorkpieceProfilesResult result = readWorkpieceProfiles(primaryPath);
    if (result.error.isOk())
    {
        return result;
    }

    return readWorkpieceProfiles(fallbackPath);
}

const IbvsWorkpieceProfile* IbvsTextConfigReader::findProfileForLegacyType(
    const std::vector<IbvsWorkpieceProfile>& profiles,
    int legacyType)
{
    for (const IbvsWorkpieceProfile& profile : profiles)
    {
        if (std::find(
                profile.legacyTypeAliases.begin(),
                profile.legacyTypeAliases.end(),
                legacyType) != profile.legacyTypeAliases.end())
        {
            return &profile;
        }
    }

    return NULL;
}

const IbvsWorkpieceProfile* IbvsTextConfigReader::findProfileById(
    const std::vector<IbvsWorkpieceProfile>& profiles,
    const std::string& workpieceId)
{
    const std::string wanted = toLower(trim(workpieceId));
    if (wanted.empty())
    {
        return NULL;
    }

    for (const IbvsWorkpieceProfile& profile : profiles)
    {
        if (toLower(profile.id) == wanted)
        {
            return &profile;
        }
    }

    return NULL;
}

} // namespace i2r_rlc::ibvs
