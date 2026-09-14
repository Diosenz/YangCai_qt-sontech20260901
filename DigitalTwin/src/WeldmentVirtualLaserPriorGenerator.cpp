/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentVirtualLaserPriorGenerator.cpp
 * @brief     基于 weldment/OCCT 的 SWv1 兼容虚拟线结构光传感器。
 * @details   按 SwLSLSensor::generate_simu_laser1 的几何语义构造激光扇形射线，
 *            与完整工件求最近交点，再使用 SENSORPARA 公式投影为图像参考线。
 */

#include "WeldmentVirtualLaserPriorGenerator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <gp.hxx>

#include <QString>

namespace i2r_rlc::digital_twin {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersToMillimeters = 1000.0;

double deg2rad(double degrees)
{
    return degrees * kPi / 180.0;
}

std::string trimString(const std::string& text)
{
    const std::size_t start = text.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const std::size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(start, end - start + 1);
}

void splitString(
    const std::string& source,
    char delimiter,
    std::vector<std::string>* output)
{
    if (output == nullptr) {
        return;
    }
    output->clear();
    std::stringstream stream(source);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        const std::string trimmed = trimString(item);
        if (!trimmed.empty()) {
            output->push_back(trimmed);
        }
    }
}

gp_Trsf stringRpyToTrsf(const std::string& rpyText)
{
    std::vector<std::string> values;
    splitString(rpyText, ',', &values);
    gp_Trsf transform;
    if (values.size() < 6U) {
        return transform;
    }

    transform.SetTranslation(
        gp_Vec(
            std::stod(values[0]),
            std::stod(values[1]),
            std::stod(values[2])));
    gp_Trsf rotationX;
    gp_Trsf rotationY;
    gp_Trsf rotationZ;
    rotationX.SetRotation(gp::OX(), deg2rad(std::stod(values[3])));
    rotationY.SetRotation(gp::OY(), deg2rad(std::stod(values[4])));
    rotationZ.SetRotation(gp::OZ(), deg2rad(std::stod(values[5])));
    transform.Multiply(rotationX);
    transform.Multiply(rotationY);
    transform.Multiply(rotationZ);
    return transform;
}

class Config final {
public:
    bool read(std::istream& input)
    {
        m_items.clear();
        std::string section;
        std::string line;
        while (std::getline(input, line)) {
            const std::string trimmed = trimString(line);
            if (trimmed.empty() || trimmed.front() == '#') {
                continue;
            }
            if (trimmed.front() == '[' && trimmed.back() == ']') {
                section = trimmed.substr(1, trimmed.size() - 2U);
                continue;
            }
            const std::size_t separator = trimmed.find(':');
            if (separator == std::string::npos) {
                continue;
            }
            m_items.push_back({
                section,
                trimString(trimmed.substr(0, separator)),
                trimString(trimmed.substr(separator + 1U))});
        }
        return true;
    }

    bool select(
        const std::string& section,
        const std::string& parameter,
        std::string* value) const
    {
        if (value == nullptr) {
            return false;
        }
        const auto match = std::find_if(
            m_items.cbegin(),
            m_items.cend(),
            [&section, &parameter](const Item& item) {
                return item.section == section && item.parameter == parameter;
            });
        if (match == m_items.cend()) {
            return false;
        }
        *value = match->value;
        return true;
    }

private:
    struct Item {
        std::string section;
        std::string parameter;
        std::string value;
    };
    std::vector<Item> m_items;
};

struct WeldmentLaserConfig {
    gp_Trsf fixPose;
    std::vector<int> seamEdgeIds;
    std::vector<std::vector<int>> positioningFaceIds;
};

bool parseWeldmentConfig(
    const std::string& path,
    WeldmentLaserConfig* result,
    QString* errorMessage)
{
    if (result == nullptr) {
        return false;
    }
    std::ifstream file(path, std::ios::in);
    if (!file.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("failed to open weldment config");
        }
        return false;
    }

    Config config;
    config.read(file);
    std::string raw;
    if (!config.select("weldment_parameters", "fix_pose/mmdeg", &raw)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("weldment fix_pose/mmdeg is missing");
        }
        return false;
    }

    try {
        result->fixPose = stringRpyToTrsf(raw);
        result->seamEdgeIds.clear();
        if (config.select("weldment_parameters", "seam_list", &raw)) {
            std::vector<std::string> seamIds;
            splitString(raw, ',', &seamIds);
            for (const std::string& seamId : seamIds) {
                result->seamEdgeIds.push_back(std::stoi(seamId));
            }
        }

        result->positioningFaceIds.clear();
        if (config.select(
                "weldment_parameters",
                "position_segment_count",
                &raw)) {
            const int segmentCount = std::stoi(raw);
            if (segmentCount < 0) {
                throw std::runtime_error("negative position_segment_count");
            }
            for (int segmentIndex = 1;
                 segmentIndex <= segmentCount;
                 ++segmentIndex) {
                std::string segmentText;
                if (!config.select(
                        "weldment_parameters",
                        "position_segment_" + std::to_string(segmentIndex),
                        &segmentText)) {
                    throw std::runtime_error(
                        "missing position_segment_"
                        + std::to_string(segmentIndex));
                }
                std::vector<std::string> fields;
                splitString(segmentText, '-', &fields);
                if (fields.size() != 4U) {
                    throw std::runtime_error(
                        "invalid position_segment_"
                        + std::to_string(segmentIndex));
                }
                std::vector<std::string> faceTexts;
                splitString(fields[2], ',', &faceTexts);
                std::vector<int> faceIds;
                faceIds.reserve(faceTexts.size());
                for (const std::string& faceText : faceTexts) {
                    const int faceId = std::stoi(faceText);
                    if (faceId < 1) {
                        throw std::runtime_error("positioning face id must be positive");
                    }
                    faceIds.push_back(faceId);
                }
                if (faceIds.empty()) {
                    throw std::runtime_error("positioning face list is empty");
                }
                result->positioningFaceIds.push_back(std::move(faceIds));
            }
        }
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("invalid weldment numeric value: %1")
                                .arg(QString::fromUtf8(exception.what()));
        }
        return false;
    }
    return true;
}

Eigen::Affine3d buildCameraFromPositionerTcp(
    const WeldmentVirtualLaserRequest& request)
{
    const WeldmentVirtualLaserCalibration& calibration = request.calibration;
    const Eigen::Matrix3d cameraRotation =
        (Eigen::AngleAxisd(
             deg2rad(calibration.cameraRxInToolDeg),
             Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(
             deg2rad(calibration.cameraRyInToolDeg),
             Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(
             deg2rad(calibration.cameraRzInToolDeg),
             Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();
    const Eigen::Affine3d cameraInTool =
        Eigen::Translation3d(
            calibration.cameraXInToolM * kMetersToMillimeters,
            calibration.cameraYInToolM * kMetersToMillimeters,
            calibration.cameraZInToolM * kMetersToMillimeters)
        * cameraRotation;

    const Eigen::Matrix3d toolRotation =
        (Eigen::AngleAxisd(
             deg2rad(request.toolRzDeg), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(
             deg2rad(request.toolRyDeg), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(
             deg2rad(request.toolRxDeg), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Affine3d toolInRobotBase =
        Eigen::Translation3d(
            request.toolXmm,
            request.toolYmm,
            request.toolZmm)
        * toolRotation;

    const Eigen::Matrix3d stationRotation =
        (Eigen::AngleAxisd(
             deg2rad(-request.stationAxis2Deg),
             Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(
             deg2rad(-request.stationAxis1Deg),
             Eigen::Vector3d::UnitY()))
            .toRotationMatrix();
    const Eigen::Affine3d robotBaseToPositionerTcp =
        Eigen::Translation3d(
            -request.positionerToolTcpInLastLinkMm.x(),
            -request.positionerToolTcpInLastLinkMm.y(),
            -request.positionerToolTcpInLastLinkMm.z())
        * Eigen::Affine3d(stationRotation)
        * request.stationBaseInRobotBase;

    return cameraInTool.inverse()
        * toolInRobotBase.inverse()
        * robotBaseToPositionerTcp.inverse();
}

DigitalTwinImagePoint2D_px projectLikeSwv1(
    const Eigen::Vector3d& pointCameraMm,
    const WeldmentVirtualLaserRequest& request,
    bool* valid)
{
    DigitalTwinImagePoint2D_px pixel;
    if (valid == nullptr) {
        return pixel;
    }
    *valid = false;

    const auto& calibration = request.calibration;
    const double x = pointCameraMm.x() / kMetersToMillimeters;
    const double y = pointCameraMm.y() / kMetersToMillimeters;
    const double z = pointCameraMm.z() / kMetersToMillimeters;
    if (z <= 1.0e-9) {
        return pixel;
    }

    const double radiusSquared = x * x + y * y;
    double u = 0.0;
    double v = 0.0;
    if (radiusSquared < 1.0e-24 || std::abs(calibration.kappa) < 1.0e-12) {
        u = calibration.focalLengthM * x / z;
        v = calibration.focalLengthM * y / z;
    } else {
        const double discriminant =
            1.0
            - 4.0 * calibration.kappa
                * calibration.focalLengthM * calibration.focalLengthM
                * radiusSquared / (z * z);
        if (discriminant < 0.0) {
            return pixel;
        }
        const double numerator = 1.0 - std::sqrt(discriminant);
        if (std::abs(x) >= std::abs(y) && std::abs(x) > 1.0e-15) {
            const double denominator =
                2.0 * calibration.kappa * calibration.focalLengthM
                * radiusSquared / (x * z);
            if (std::abs(denominator) < 1.0e-24) {
                return pixel;
            }
            u = numerator / denominator;
            v = u * y / x;
        } else if (std::abs(y) > 1.0e-15) {
            const double denominator =
                2.0 * calibration.kappa * calibration.focalLengthM
                * radiusSquared / (y * z);
            if (std::abs(denominator) < 1.0e-24) {
                return pixel;
            }
            v = numerator / denominator;
            u = v * x / y;
        }
    }

    const double imageX = calibration.cxPixel + u / calibration.sxM;
    const double imageY = calibration.cyPixel + v / calibration.syM;
    if (!std::isfinite(imageX) || !std::isfinite(imageY)
        || imageX < 0.0 || imageX >= static_cast<double>(request.imageWidth)
        || imageY < 0.0 || imageY >= static_cast<double>(request.imageHeight)) {
        return pixel;
    }

    // SWv1 在保存虚拟线前转换为 int；保留同样的像素取整语义。
    pixel.x_px = static_cast<double>(static_cast<int>(imageX));
    pixel.y_px = static_cast<double>(static_cast<int>(imageY));
    *valid = true;
    return pixel;
}

TopoDS_Edge findEdgeById(const TopoDS_Shape& shape, int edgeId)
{
    if (edgeId < 1) {
        return {};
    }
    TopExp_Explorer explorer(shape, TopAbs_EDGE);
    int currentId = 1;
    for (; explorer.More(); explorer.Next(), ++currentId) {
        if (currentId == edgeId) {
            return TopoDS::Edge(explorer.Current());
        }
    }
    return {};
}

std::vector<gp_Pnt> intersectEdgeWithPlane(
    const TopoDS_Edge& edge,
    const gp_Pln& plane)
{
    std::vector<gp_Pnt> points;
    if (edge.IsNull()) {
        return points;
    }
    try {
        BRepAlgoAPI_Section section(edge, plane);
        section.Build();
        if (!section.IsDone()) {
            return points;
        }
        TopExp_Explorer explorer(section.Shape(), TopAbs_VERTEX);
        for (; explorer.More(); explorer.Next()) {
            points.push_back(BRep_Tool::Pnt(TopoDS::Vertex(explorer.Current())));
        }
    } catch (...) {
        points.clear();
    }
    return points;
}

} // namespace

struct WeldmentVirtualLaserPriorGenerator::Impl {
    bool ensureModelLoaded(const std::string& path, QString* errorMessage)
    {
        if (path == cachedPath && !shapeInPositionerTcp.IsNull()) {
            return true;
        }

        WeldmentLaserConfig parsedConfig;
        if (!parseWeldmentConfig(path, &parsedConfig, errorMessage)) {
            return false;
        }

        STEPControl_Reader reader;
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("failed to read STEP model: %1")
                                    .arg(QString::fromStdString(path));
            }
            return false;
        }
        reader.TransferRoots();
        const TopoDS_Shape model = reader.OneShape();
        if (model.IsNull()) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("STEP model shape is null");
            }
            return false;
        }

        BRepBuilderAPI_Transform transform(model, parsedConfig.fixPose);
        const TopoDS_Shape transformed = transform.Shape();
        if (transformed.IsNull()) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("failed to apply weldment fix_pose");
            }
            return false;
        }

        cachedPath = path;
        config = std::move(parsedConfig);
        shapeInPositionerTcp = transformed;
        faceMap.Clear();
        TopExp::MapShapes(shapeInPositionerTcp, TopAbs_FACE, faceMap);
        for (const std::vector<int>& faceIds : config.positioningFaceIds) {
            for (const int faceId : faceIds) {
                if (faceId > faceMap.Extent()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = QStringLiteral(
                            "weldment positioning face id %1 is outside STEP face range 1..%2")
                                                .arg(faceId)
                                                .arg(faceMap.Extent());
                    }
                    cachedPath.clear();
                    shapeInPositionerTcp.Nullify();
                    faceMap.Clear();
                    return false;
                }
            }
        }
        return true;
    }

    std::string cachedPath;
    WeldmentLaserConfig config;
    TopoDS_Shape shapeInPositionerTcp;
    TopTools_IndexedMapOfShape faceMap;
};

WeldmentVirtualLaserPriorGenerator::WeldmentVirtualLaserPriorGenerator()
    : m_impl(std::make_unique<Impl>())
{
}

WeldmentVirtualLaserPriorGenerator::~WeldmentVirtualLaserPriorGenerator() = default;

DigitalTwinVisionLaserObservation WeldmentVirtualLaserPriorGenerator::generate(
    const WeldmentVirtualLaserRequest& request) const
{
    DigitalTwinVisionLaserObservation observation =
        makeInvalidVisionLaserObservation(
            -1,
            QStringLiteral("weldment virtual laser prior unavailable"));

    const auto& calibration = request.calibration;
    if (!calibration.valid) {
        observation.message = QStringLiteral("SENSORPARA calibration is invalid");
        return observation;
    }
    if (request.weldmentPath.empty()) {
        observation.message = QStringLiteral("weldment path is empty");
        return observation;
    }
    if (std::abs(calibration.focalLengthM) < 1.0e-12
        || std::abs(calibration.sxM) < 1.0e-12
        || std::abs(calibration.syM) < 1.0e-12
        || std::abs(calibration.laserPlaneC) < 1.0e-12) {
        observation.message = QStringLiteral("SENSORPARA focal/pixel/laser parameters are invalid");
        return observation;
    }
    if (request.planeAngleStepDeg <= 0.0
        || request.planeAngleEndDeg <= request.planeAngleStartDeg
        || request.maxRayDistanceMm <= 0.0) {
        observation.message = QStringLiteral("virtual laser fan parameters are invalid");
        return observation;
    }

    QString modelError;
    if (!m_impl->ensureModelLoaded(request.weldmentPath, &modelError)) {
        observation.message = modelError;
        return observation;
    }

    const Eigen::Affine3d cameraInPositionerTcp =
        buildCameraFromPositionerTcp(request);
    const Eigen::Affine3d positionerTcpFromCamera =
        cameraInPositionerTcp.inverse();

    const std::vector<int>* targetFaceIds = nullptr;
    int targetFaceCount = 0;
    if (request.positioningSegmentIndex > 0) {
        const std::size_t requestedIndex = static_cast<std::size_t>(
            request.positioningSegmentIndex - 1);
        if (requestedIndex >= m_impl->config.positioningFaceIds.size()) {
            observation.message = QStringLiteral(
                "positioning segment %1 is outside weldment range 1..%2")
                                      .arg(request.positioningSegmentIndex)
                                      .arg(static_cast<int>(
                                          m_impl->config.positioningFaceIds.size()));
            return observation;
        }
        targetFaceIds = &m_impl->config.positioningFaceIds[requestedIndex];
        targetFaceCount = static_cast<int>(
            targetFaceIds->size());
    }

    // SENSORPARA 的激光平面方程是 a*x+b*y+c*z+1=0，坐标单位为 m。
    // SWv1 以相机原点在该平面上的投影点作为扇形射线原点。
    const Eigen::Vector3d normalCamera(
        calibration.laserPlaneA,
        calibration.laserPlaneB,
        calibration.laserPlaneC);
    const double normalSquared = normalCamera.squaredNorm();
    if (normalSquared < 1.0e-18) {
        observation.message = QStringLiteral("laser plane normal is zero");
        return observation;
    }
    const Eigen::Vector3d laserOriginCameraMm =
        (-normalCamera / normalSquared) * kMetersToMillimeters;
    const Eigen::Vector3d planeOnCameraZMm(
        0.0,
        0.0,
        (-1.0 / calibration.laserPlaneC) * kMetersToMillimeters);
    Eigen::Vector3d referenceDirectionCamera =
        planeOnCameraZMm - laserOriginCameraMm;
    if (planeOnCameraZMm.z() < 0.0) {
        referenceDirectionCamera = -referenceDirectionCamera;
    }
    if (referenceDirectionCamera.norm() < 1.0e-12) {
        observation.message = QStringLiteral("virtual laser reference direction is zero");
        return observation;
    }

    const Eigen::Vector3d laserOriginPositioner =
        positionerTcpFromCamera * laserOriginCameraMm;
    const Eigen::Vector3d normalPositioner =
        (positionerTcpFromCamera.linear() * normalCamera).normalized();
    const Eigen::Vector3d referenceDirectionPositioner =
        (positionerTcpFromCamera.linear() * referenceDirectionCamera).normalized();
    const gp_Pnt rayOrigin(
        laserOriginPositioner.x(),
        laserOriginPositioner.y(),
        laserOriginPositioner.z());

    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(m_impl->shapeInPositionerTcp, 10.0e-3);

    std::vector<gp_Pnt> laserPointsPositioner;
    const int rayCount = static_cast<int>(std::ceil(
        (request.planeAngleEndDeg - request.planeAngleStartDeg)
        / request.planeAngleStepDeg));
    laserPointsPositioner.reserve(static_cast<std::size_t>(rayCount));
    for (double angle = request.planeAngleStartDeg;
         angle < request.planeAngleEndDeg;
         angle += request.planeAngleStepDeg) {
        const Eigen::Vector3d direction =
            (Eigen::AngleAxisd(deg2rad(angle), normalPositioner)
             * referenceDirectionPositioner)
                .normalized();
        const gp_Lin ray(
            rayOrigin,
            gp_Dir(direction.x(), direction.y(), direction.z()));
        intersector.Perform(ray, 0.0, request.maxRayDistanceMm);
        if (!intersector.IsDone() || intersector.NbPnt() <= 0) {
            continue;
        }

        Standard_Real nearestParameter = std::numeric_limits<Standard_Real>::max();
        Standard_Integer nearestIntersectionIndex = 0;
        gp_Pnt nearestPoint;
        for (Standard_Integer index = 1; index <= intersector.NbPnt(); ++index) {
            const Standard_Real parameter = intersector.WParameter(index);
            if (parameter >= 0.0 && parameter < nearestParameter) {
                nearestParameter = parameter;
                nearestIntersectionIndex = index;
                nearestPoint = intersector.Pnt(index);
            }
        }
        if (nearestIntersectionIndex <= 0) {
            continue;
        }

        // SWv1 先在整件模型上取每条射线的最近交点，再检查该交点所属 face
        // 是否属于当前 position_segment；不能把非定位面移除后继续命中后方表面。
        if (targetFaceIds != nullptr) {
            const int hitFaceId = m_impl->faceMap.FindIndex(
                intersector.Face(nearestIntersectionIndex));
            if (std::find(
                    targetFaceIds->cbegin(),
                    targetFaceIds->cend(),
                    hitFaceId)
                == targetFaceIds->cend()) {
                continue;
            }
        }
        laserPointsPositioner.push_back(nearestPoint);
    }

    std::vector<DigitalTwinImagePoint2D_px> laserLine;
    laserLine.reserve(laserPointsPositioner.size());
    double minimumCameraZMm = std::numeric_limits<double>::max();
    double maximumCameraZMm = std::numeric_limits<double>::lowest();
    int frontCameraHitCount = 0;
    for (const gp_Pnt& point : laserPointsPositioner) {
        const Eigen::Vector3d pointCamera =
            cameraInPositionerTcp
            * Eigen::Vector3d(point.X(), point.Y(), point.Z());
        minimumCameraZMm = std::min(minimumCameraZMm, pointCamera.z());
        maximumCameraZMm = std::max(maximumCameraZMm, pointCamera.z());
        if (pointCamera.z() > 0.0) {
            ++frontCameraHitCount;
        }
        bool projected = false;
        const auto pixel = projectLikeSwv1(pointCamera, request, &projected);
        if (projected) {
            laserLine.push_back(pixel);
        }
    }
    if (laserLine.size() < 2U) {
        const QString cameraZRange = laserPointsPositioner.empty()
            ? QStringLiteral("n/a")
            : QStringLiteral("%1..%2")
                  .arg(minimumCameraZMm, 0, 'f', 3)
                  .arg(maximumCameraZMm, 0, 'f', 3);
        observation.message = QStringLiteral(
            "SWv1 ray fan has insufficient visible hits: segment=%1, targetFaces=%2, rays=%3, modelHits=%4, frontHits=%5, cameraZmm=%6, imageHits=%7")
                                  .arg(request.positioningSegmentIndex)
                                  .arg(targetFaceCount)
                                  .arg(rayCount)
                                  .arg(static_cast<int>(laserPointsPositioner.size()))
                                  .arg(frontCameraHitCount)
                                  .arg(cameraZRange)
                                  .arg(static_cast<int>(laserLine.size()));
        return observation;
    }

    const gp_Pln laserPlane(
        rayOrigin,
        gp_Dir(normalPositioner.x(), normalPositioner.y(), normalPositioner.z()));
    DigitalTwinImagePoint2D_px weldPoint;
    bool hasWeldPoint = false;
    double nearestSeamDistanceSquared = std::numeric_limits<double>::max();
    int selectedSeamEdgeId = -1;
    for (const int edgeId : m_impl->config.seamEdgeIds) {
        const TopoDS_Edge edge = findEdgeById(m_impl->shapeInPositionerTcp, edgeId);
        for (const gp_Pnt& seamPoint : intersectEdgeWithPlane(edge, laserPlane)) {
            const double distanceSquared = rayOrigin.SquareDistance(seamPoint);
            if (distanceSquared >= nearestSeamDistanceSquared) {
                continue;
            }
            const Eigen::Vector3d pointCamera =
                cameraInPositionerTcp
                * Eigen::Vector3d(seamPoint.X(), seamPoint.Y(), seamPoint.Z());
            bool projected = false;
            const auto pixel = projectLikeSwv1(pointCamera, request, &projected);
            if (projected) {
                nearestSeamDistanceSquared = distanceSquared;
                weldPoint = pixel;
                hasWeldPoint = true;
                selectedSeamEdgeId = edgeId;
            }
        }
    }
    if (!hasWeldPoint) {
        double sumX = 0.0;
        double sumY = 0.0;
        for (const auto& point : laserLine) {
            sumX += point.x_px;
            sumY += point.y_px;
        }
        weldPoint.x_px = sumX / static_cast<double>(laserLine.size());
        weldPoint.y_px = sumY / static_cast<double>(laserLine.size());
    }

    observation.valid = true;
    observation.frameId = -1;
    observation.laserLineImagePoints_px = std::move(laserLine);
    observation.hasWeldPointImage_px = true;
    observation.weldPointImage_px = weldPoint;
    observation.seamIndex = selectedSeamEdgeId;
    observation.positioningSegmentIndex = request.positioningSegmentIndex;
    observation.message = QStringLiteral(
        "SWv1-compatible OCCT virtual laser ready: segment=%1, targetFaces=%2, rays=%3, modelHits=%4, imageHits=%5, modelCached=%6")
                              .arg(request.positioningSegmentIndex)
                              .arg(targetFaceCount)
                              .arg(rayCount)
                              .arg(static_cast<int>(laserPointsPositioner.size()))
                              .arg(static_cast<int>(observation.laserLineImagePoints_px.size()))
                              .arg(QString::fromStdString(m_impl->cachedPath));
    return observation;
}

} // namespace i2r_rlc::digital_twin
