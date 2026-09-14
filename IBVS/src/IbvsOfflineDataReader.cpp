/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      IbvsOfflineDataReader.cpp
 * @brief     IBVS 离线文本数据读取实现。
 * @details   本文件解析特征、ROI、机器人位移、相机位姿、速度和初始雅可比文本文件。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 6 月 8 日
 */

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "ibvs/IbvsOfflineDataReader.h"

namespace i2r_rlc::ibvs {

namespace {

std::vector<double> parseNumbers(const std::string& line)
{
    const std::size_t firstContent = line.find_first_not_of(" \t\r\n");
    if (firstContent == std::string::npos || line[firstContent] == '#')
    {
        return {};
    }

    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), ';', ' ');

    std::istringstream stream(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value)
    {
        values.push_back(value);
    }
    return values;
}

IbvsError makeOpenError(const std::string& path)
{
    return {
        IbvsStatus::FILE_OPEN_FAILED,
        "failed to open file: " + path
    };
}

std::ifstream openInputFile(const std::string& path)
{
    return std::ifstream(std::filesystem::u8path(path));
}

} // namespace

IbvsDataSeries<IbvsFeatureVector>
IbvsOfflineDataReader::readFeatureCoordinates(const std::string& path)
{
    IbvsDataSeries<IbvsFeatureVector> result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        if (values.size() < 4)
        {
            continue;
        }

        const std::size_t valueOffset = values.size() == 4 ? 0 : 1;
        if (values.size() < valueOffset + 4)
        {
            continue;
        }

        IbvsFeatureVector feature;
        feature << values[valueOffset],
            values[valueOffset + 1],
            values[valueOffset + 2],
            values[valueOffset + 3];
        result.values.push_back(feature);
    }

    result.error = result.values.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no feature coordinates found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

IbvsDataSeries<Eigen::Vector3d>
IbvsOfflineDataReader::readRobotTranslations(const std::string& path)
{
    IbvsDataSeries<Eigen::Vector3d> result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        if (values.size() < 3)
        {
            continue;
        }
        result.values.push_back(Eigen::Vector3d(values[0], values[1], values[2]));
    }

    result.error = result.values.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no robot translations found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

IbvsDataSeries<IbvsPoseVector>
IbvsOfflineDataReader::readCameraPoses(const std::string& path)
{
    IbvsDataSeries<IbvsPoseVector> result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        if (values.size() < 6)
        {
            continue;
        }

        IbvsPoseVector pose;
        pose << values[0], values[1], values[2],
            values[3], values[4], values[5];
        result.values.push_back(pose);
    }

    result.error = result.values.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no camera poses found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

IbvsJacobianFileResult IbvsOfflineDataReader::readInitialJacobian(
    const std::string& path)
{
    IbvsJacobianFileResult result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::vector<double> allValues;
    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        if (values.empty())
        {
            continue;
        }

        allValues.insert(allValues.end(), values.begin(), values.end());
        if (allValues.size() >= 12)
        {
            break;
        }
    }

    if (allValues.size() < 12)
    {
        result.error = {
            IbvsStatus::PARSE_ERROR,
            "initial jacobian file must contain at least 12 numbers"
        };
        return result;
    }

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            result.jacobian(row, col) = allValues[static_cast<std::size_t>(row * 3 + col)];
        }
    }
    result.error = { IbvsStatus::OK, std::string() };
    return result;
}

IbvsDataSeries<PixelRoi> IbvsOfflineDataReader::readRois(
    const std::string& path)
{
    IbvsDataSeries<PixelRoi> result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        if (values.size() < 4)
        {
            continue;
        }

        const std::size_t valueOffset = values.size() == 4 ? 0 : 1;
        if (values.size() < valueOffset + 4)
        {
            continue;
        }

        PixelRoi roi;
        roi.x = static_cast<int>(values[valueOffset]);
        roi.y = static_cast<int>(values[valueOffset + 1]);
        roi.width = static_cast<int>(values[valueOffset + 2]);
        roi.height = static_cast<int>(values[valueOffset + 3]);
        if (roi.isValid())
        {
            result.values.push_back(roi);
        }
    }

    result.error = result.values.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no valid ROI records found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

IbvsDataSeries<IbvsVelocityVector>
IbvsOfflineDataReader::readVelocitySeries(const std::string& path)
{
    IbvsDataSeries<IbvsVelocityVector> result;
    std::ifstream file = openInputFile(path);
    if (!file.is_open())
    {
        result.error = makeOpenError(path);
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        const std::vector<double> values = parseNumbers(line);
        const std::size_t valueOffset = values.size() >= 7 ? 1 : 0;
        if (values.size() < valueOffset + 3)
        {
            continue;
        }
        result.values.push_back(Eigen::Vector3d(
            values[valueOffset],
            values[valueOffset + 1],
            values[valueOffset + 2]));
    }

    result.error = result.values.empty()
        ? IbvsError{ IbvsStatus::PARSE_ERROR, "no velocity records found" }
        : IbvsError{ IbvsStatus::OK, std::string() };
    return result;
}

} // namespace i2r_rlc::ibvs
