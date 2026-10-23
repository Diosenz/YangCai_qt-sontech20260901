/** @file OnlineSeamCandidateGeometry.cpp
 * @brief CAD 候选重求交：冻结曝光平面，独立计算当前/候选几何。
 */
#include "OnlineSeamCandidateGeometry.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

#include <BRep_Tool.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineCurve.hxx>
#include <GeomAPI_ExtremaCurveSurface.hxx>
#include <GeomAPI_IntCS.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <Eigen/Geometry>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>

namespace i2r_rlc::digital_twin {
namespace {

using Point = localization::Point3D_mm;
using Matrix = Eigen::Matrix4d;

/** @brief 仅保留场景值树；解析预算避免损坏 XML 导致无界递归/分配。 */
struct SceneNode {
    QString name;
    QString text;
    std::vector<SceneNode> children;
};

const SceneNode *child(const SceneNode &node, const QString &name)
{
    const SceneNode *result = nullptr;
    for (const auto &entry : node.children) {
        if (entry.name != name) { continue; }
        if (result != nullptr) { return nullptr; }
        result = &entry;
    }
    return result;
}

bool readNode(QXmlStreamReader &xml, SceneNode &node, int depth, int &count)
{
    if (depth > 32 || ++count > 100000 || !xml.isStartElement()) { return false; }
    node.name = xml.name().toString();
    while (!xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement()) {
            SceneNode entry;
            if (!readNode(xml, entry, depth + 1, count)) { return false; }
            node.children.push_back(std::move(entry));
        } else if (xml.isCharacters()) {
            node.text += xml.text();
        } else if (xml.isEndElement()) {
            node.text = node.text.trimmed();
            return true;
        } else if (xml.isDTD() || xml.isEntityReference()) {
            return false;
        }
    }
    return false;
}

bool positiveId(const SceneNode *node, int &value)
{
    if (node == nullptr || !node->children.empty()) { return false; }
    bool ok = false;
    value = node->text.toInt(&ok);
    return ok && value > 0;
}

/** @brief 仅按同一场景中的实际对象身份读取路径/边列表，不使用工件名称猜测。 */
bool parseBinding(const OnlineSeamSceneBinding &binding, QString &modelFile,
    std::vector<int> &seamIds, QString &message)
{
    QFile file(binding.sceneFile);
    if (binding.workpieceId <= 0 || binding.parentId <= 0 || binding.sensorId <= 0
        || !file.open(QIODevice::ReadOnly) || file.size() > 16 * 1024 * 1024) {
        message = QStringLiteral("场景路径、对象 ID 或 XML 文件不可用。");
        return false;
    }
    QXmlStreamReader xml(&file);
    if (!xml.readNextStartElement()) { return false; }
    SceneNode root;
    int count = 0;
    if (!readNode(xml, root, 0, count) || xml.hasError()) {
        message = QStringLiteral("场景 XML 损坏或超过解析预算。");
        return false;
    }
    while (!xml.atEnd()) { xml.readNext(); }
    const SceneNode *scene = child(root, QStringLiteral("scene"));
    if (xml.hasError() || root.name != QStringLiteral("root") || scene == nullptr) {
        message = QStringLiteral("场景必须包含唯一 root/scene。");
        return false;
    }
    const SceneNode *workpiece = nullptr;
    const SceneNode *sensor = nullptr;
    for (const auto &object : scene->children) {
        const auto *base = child(object, QStringLiteral("base_object"));
        int id = -1;
        if (base == nullptr || !positiveId(child(*base, QStringLiteral("id")), id)) { continue; }
        if (id == binding.workpieceId) {
            if (workpiece != nullptr || object.name != QStringLiteral("OBJ_SHAPE")) { return false; }
            workpiece = &object;
        }
        if (id == binding.sensorId) {
            if (sensor != nullptr || object.name != QStringLiteral("SENSOR_LSL")) { return false; }
            sensor = &object;
        }
    }
    if (workpiece == nullptr || sensor == nullptr) {
        message = QStringLiteral("场景缺少指定工件或线结构光传感器。");
        return false;
    }
    const auto *base = child(*workpiece, QStringLiteral("base_object"));
    int parentId = -1;
    if (!positiveId(child(*base, QStringLiteral("parent_id")), parentId)
        || parentId != binding.parentId) {
        message = QStringLiteral("工件父级与当前连接配置不一致。");
        return false;
    }
    const auto *shape = child(*workpiece, QStringLiteral("shape"));
    const auto *model = shape != nullptr ? child(*shape, QStringLiteral("model")) : nullptr;
    const auto *path = model != nullptr ? child(*model, QStringLiteral("path")) : nullptr;
    // 未支持的模型局部变换不能默默忽略；对象位姿由每次运行时快照显式提供。
    if (path == nullptr || path->text.isEmpty() || !path->children.empty()
        || model->children.size() != 1U || shape->children.size() != 1U) {
        message = QStringLiteral("模型路径缺失或包含未支持的模型局部变换。");
        return false;
    }
    const auto *camera = child(*sensor, QStringLiteral("lsl_camera"));
    int sensorWorkpiece = -1;
    if (camera == nullptr
        || !positiveId(child(*camera, QStringLiteral("workpiece")), sensorWorkpiece)
        || sensorWorkpiece != binding.workpieceId) {
        message = QStringLiteral("传感器绑定的工件与当前配置不一致。");
        return false;
    }
    for (const auto &entry : camera->children) {
        if (entry.name != QStringLiteral("weld_seam")) { continue; }
        int id = -1;
        if (!positiveId(&entry, id)
            || std::find(seamIds.begin(), seamIds.end(), id) != seamIds.end()) {
            message = QStringLiteral("焊缝边号无效或重复。");
            return false;
        }
        seamIds.push_back(id);
    }
    if (seamIds.empty()) {
        message = QStringLiteral("场景没有明确焊缝边号。");
        return false;
    }
    const QDir sceneDirectory = QFileInfo(binding.sceneFile).absoluteDir();
    modelFile = QFileInfo(sceneDirectory.absoluteFilePath(path->text)).canonicalFilePath();
    if (modelFile.isEmpty() || !QFileInfo(modelFile).isFile()) {
        message = QStringLiteral("场景实际 STEP 路径不存在。");
        return false;
    }
    return true;
}

Matrix matrix(const localization::RigidTransform &value)
{
    Matrix result;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result(row, column) = value.rowMajor[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    return result;
}

bool rigid(const Matrix &value)
{
    if (!value.allFinite() || (value.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1.0e-8) { return false; }
    const Eigen::Matrix3d rotation = value.block<3, 3>(0, 0);
    return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1.0e-7
        && std::abs(rotation.determinant() - 1.0) <= 1.0e-7;
}

Eigen::Vector3d vector(const Point &value)
{
    return {value.x_mm, value.y_mm, value.z_mm};
}

Point point(const Eigen::Vector3d &value)
{
    return {value.x(), value.y(), value.z()};
}

struct SeamEdge {
    int id = -1;
    TopoDS_Edge edge;
};

struct Intersection {
    OnlineSeamGeometryStatus status = OnlineSeamGeometryStatus::NoIntersection;
    int edgeId = -1;
    Eigen::Vector3d modelPoint = Eigen::Vector3d::Zero();
};

/** @brief 求有限拓扑边与模型坐标平面的交点；相切/共面不构成唯一可观测截面。 */
Intersection intersect(const std::vector<SeamEdge> &edges, const Matrix &parentFromModel,
    const Eigen::Vector4d &parentPlane, int requestedId)
{
    Intersection result;
    const Eigen::Vector4d modelPlane = parentFromModel.transpose() * parentPlane;
    const Eigen::Vector3d normal = modelPlane.head<3>();
    const gp_Pln plane(normal.x(), normal.y(), normal.z(), modelPlane.w());
    const Handle(Geom_Plane) surface = new Geom_Plane(plane);
    int intersections = 0;
    for (const auto &entry : edges) {
        if (requestedId > 0 && requestedId != entry.id) { continue; }
        Standard_Real first = 0.0;
        Standard_Real last = 0.0;
        const Handle(Geom_Curve) original = BRep_Tool::Curve(entry.edge, first, last);
        if (original.IsNull() || !std::isfinite(first) || !std::isfinite(last)
            || last <= first) {
            result.status = OnlineSeamGeometryStatus::DegenerateIntersection;
            return result;
        }
        const Handle(Geom_TrimmedCurve) bounded = new Geom_TrimmedCurve(original, first, last);
        GeomAPI_IntCS checked(bounded, surface);
        if (!checked.IsDone() || checked.NbSegments() > 0) {
            result.status = OnlineSeamGeometryStatus::DegenerateIntersection;
            return result;
        }
        if (checked.NbPoints() == 0) {
            // OCCT 的共面曲线可能给出“完成但零点/零段”；用一般曲线/平面
            // 平行关系补充分型，不能把无限多个截点当作有限边没有截点。
            const GeomAPI_ExtremaCurveSurface distance(bounded, surface);
            bool coplanar = distance.Extrema().IsDone() && distance.IsParallel()
                && plane.Distance(bounded->Value(0.5 * (first + last))) <= 1.0e-7;
            if (!coplanar) {
                // 样条的无限平行极值未必被 Extrema 标记；精确转换后全控制点
                // 共面是整条有理样条共面的充分条件，不用有限采样猜测曲线形状。
                const Handle(Geom_BSplineCurve) spline = GeomConvert::CurveToBSplineCurve(bounded);
                coplanar = !spline.IsNull() && spline->NbPoles() > 0;
                for (int pole = 1; coplanar && pole <= spline->NbPoles(); ++pole) {
                    coplanar = plane.Distance(spline->Pole(pole)) <= 1.0e-7;
                }
            }
            if (coplanar) {
                result.status = OnlineSeamGeometryStatus::DegenerateIntersection;
                return result;
            }
        }
        for (int index = 1; index <= checked.NbPoints(); ++index) {
            Standard_Real u = 0.0;
            Standard_Real v = 0.0;
            Standard_Real parameter = 0.0;
            checked.Parameters(index, u, v, parameter);
            gp_Pnt position;
            gp_Vec tangent;
            bounded->D1(parameter, position, tangent);
            const Eigen::Vector3d derivative(tangent.X(), tangent.Y(), tangent.Z());
            // 这里只使用数值奇异容差，不对曲率、焊缝形状或修正方向作业务特化。
            if (!derivative.allFinite() || derivative.norm() <= 1.0e-12
                || std::abs(normal.dot(derivative)) <= 1.0e-6 * normal.norm() * derivative.norm()) {
                result.status = OnlineSeamGeometryStatus::DegenerateIntersection;
                return result;
            }
            const auto &hit = checked.Point(index);
            const Eigen::Vector3d candidate(hit.X(), hit.Y(), hit.Z());
            if (intersections > 0 && entry.id == result.edgeId
                && (candidate - result.modelPoint).norm() <= 1.0e-6) { continue; }
            ++intersections;
            if (intersections > 1) {
                result.status = OnlineSeamGeometryStatus::AmbiguousIntersection;
                return result;
            }
            result.modelPoint = candidate;
            result.edgeId = entry.id;
        }
    }
    if (intersections == 1) { result.status = OnlineSeamGeometryStatus::Succeeded; }
    return result;
}

Point inParent(const Matrix &parentFromModel, const Eigen::Vector3d &modelPoint)
{
    return point(parentFromModel.block<3, 3>(0, 0) * modelPoint
        + parentFromModel.block<3, 1>(0, 3));
}

} // namespace

struct COnlineSeamCandidateGeometry::Impl {
    OnlineSeamSceneBinding binding;
    QString modelFile;
    qint64 sceneSize = -1;
    qint64 modelSize = -1;
    QDateTime sceneModified;
    QDateTime modelModified;
    std::vector<SeamEdge> edges;
    OnlineSeamGeometryLoadResult loadResult;

    bool filesUnchanged() const
    {
        const QFileInfo scene(binding.sceneFile);
        const QFileInfo model(modelFile);
        return loadResult.loaded && scene.isFile() && model.isFile()
            && scene.size() == sceneSize && model.size() == modelSize
            && scene.lastModified() == sceneModified && model.lastModified() == modelModified;
    }
};

COnlineSeamCandidateGeometry::COnlineSeamCandidateGeometry()
    : m_impl(std::make_unique<Impl>())
{
}

COnlineSeamCandidateGeometry::~COnlineSeamCandidateGeometry() = default;

OnlineSeamGeometryLoadResult COnlineSeamCandidateGeometry::load(
    const OnlineSeamSceneBinding &binding)
{
    if (m_impl->binding.sceneFile == binding.sceneFile
        && m_impl->binding.workpieceId == binding.workpieceId
        && m_impl->binding.parentId == binding.parentId
        && m_impl->binding.sensorId == binding.sensorId && m_impl->filesUnchanged()) {
        return m_impl->loadResult;
    }
    clear();
    OnlineSeamGeometryLoadResult result;
    try {
        if (!parseBinding(binding, result.modelFile, result.seamEdgeIds, result.message)) {
            if (result.message.isEmpty()) { result.message = QStringLiteral("场景身份或模型绑定不唯一。"); }
            return result;
        }
        const QFileInfo sceneBefore(binding.sceneFile);
        const QFileInfo modelBefore(result.modelFile);
        STEPControl_Reader reader;
        const QByteArray encodedPath = QFile::encodeName(result.modelFile);
        if (reader.ReadFile(encodedPath.constData()) != IFSelect_RetDone
            || reader.TransferRoots() <= 0) {
            result.message = QStringLiteral("实际 STEP 加载失败，未保留旧缓存。");
            return result;
        }
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) { return result; }
        // 显式沿用现有 Qt/旧传感器的 1-based TopExp_Explorer 出现序。
        // 边号存在不证明当前服务版本一致，evaluate 仍须复现当前服务交点。
        std::vector<SeamEdge> edges;
        int id = 0;
        for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
            ++id;
            if (std::find(result.seamEdgeIds.begin(), result.seamEdgeIds.end(), id)
                != result.seamEdgeIds.end()) {
                edges.push_back({id, TopoDS::Edge(explorer.Current())});
            }
        }
        if (edges.size() != result.seamEdgeIds.size()) {
            result.message = QStringLiteral("场景焊缝边号超出实际 STEP 范围。");
            return result;
        }
        const QFileInfo sceneAfter(binding.sceneFile);
        const QFileInfo modelAfter(result.modelFile);
        if (sceneBefore.size() != sceneAfter.size() || sceneBefore.lastModified() != sceneAfter.lastModified()
            || modelBefore.size() != modelAfter.size() || modelBefore.lastModified() != modelAfter.lastModified()) {
            result.message = QStringLiteral("加载期间场景或 STEP 被修改。");
            return result;
        }
        m_impl->binding = binding;
        m_impl->modelFile = result.modelFile;
        m_impl->sceneSize = sceneAfter.size();
        m_impl->modelSize = modelAfter.size();
        m_impl->sceneModified = sceneAfter.lastModified();
        m_impl->modelModified = modelAfter.lastModified();
        m_impl->edges = std::move(edges);
        result.loaded = true;
        result.message = QStringLiteral("只读 CAD 已加载；每帧仍须当前参考复现与唯一分支校核。");
        m_impl->loadResult = result;
        return result;
    } catch (const Standard_Failure &failure) {
        result.message = QStringLiteral("OCCT 加载异常：%1").arg(QString::fromUtf8(failure.GetMessageString()));
    } catch (const std::exception &failure) {
        result.message = QStringLiteral("CAD 加载异常：%1").arg(QString::fromUtf8(failure.what()));
    } catch (...) {
        result.message = QStringLiteral("CAD 加载发生未知异常。");
    }
    clear();
    return result;
}

OnlineSeamGeometryResult COnlineSeamCandidateGeometry::evaluate(
    const OnlineSeamGeometryRequest &request) const
{
    OnlineSeamGeometryResult result;
    try {
        if (!m_impl->filesUnchanged()) {
            result.message = QStringLiteral("CAD 未加载或文件已变化。");
            return result;
        }
        const Matrix current = matrix(request.parentFromWorkpiece);
        Eigen::Vector4d plane(request.parentLaserPlane.nx, request.parentLaserPlane.ny,
            request.parentLaserPlane.nz, request.parentLaserPlane.dMm);
        const double length = plane.head<3>().norm();
        if (!rigid(current) || !plane.allFinite() || !std::isfinite(length) || length <= 1.0e-12
            || !vector(request.currentVirtualWeld).allFinite() || !vector(request.realWeld).allFinite()
            || !std::isfinite(request.referenceToleranceMm) || request.referenceToleranceMm <= 0.0
            || (request.seamEdgeId != -1 && request.seamEdgeId <= 0)) {
            result.status = OnlineSeamGeometryStatus::InvalidInput;
            result.message = QStringLiteral("当前位姿、激光平面、参考点或容差无效。");
            return result;
        }
        if (request.seamEdgeId > 0 && std::none_of(m_impl->edges.begin(), m_impl->edges.end(),
                [&request](const SeamEdge &edge) { return edge.id == request.seamEdgeId; })) {
            result.status = OnlineSeamGeometryStatus::InvalidBinding;
            result.message = QStringLiteral("指定焊缝身份不属于场景传感器配置。");
            return result;
        }
        plane /= length;
        const Intersection before = intersect(m_impl->edges, current, plane, request.seamEdgeId);
        result.status = before.status;
        if (before.status != OnlineSeamGeometryStatus::Succeeded) {
            result.message = QStringLiteral("当前 CAD 不存在唯一非退化交点，未建立几何身份。");
            return result;
        }
        result.seamEdgeId = before.edgeId;
        result.beforeIntersection = inParent(current, before.modelPoint);
        result.referenceDifferenceMm = (vector(result.beforeIntersection) - vector(request.currentVirtualWeld)).norm();
        if (result.referenceDifferenceMm > request.referenceToleranceMm) {
            result.status = OnlineSeamGeometryStatus::ReferenceMismatch;
            result.message = QStringLiteral("当前 CAD 交点不能复现同帧 RobotSim 参考。");
            return result;
        }
        result.referenceVerified = true;
        result.beforeErrorMm = (vector(result.beforeIntersection) - vector(request.realWeld)).norm();
        const Matrix correction = matrix(request.correction);
        if (!rigid(correction)) {
            result.status = OnlineSeamGeometryStatus::InvalidInput;
            result.message = QStringLiteral("当前参考已确认，但候选并非有限刚体变换。");
            return result;
        }
        const Matrix candidate = correction * current;
        const Intersection after = intersect(m_impl->edges, candidate, plane, before.edgeId);
        result.status = after.status;
        if (after.status != OnlineSeamGeometryStatus::Succeeded) {
            result.message = QStringLiteral("已确认焊缝的候选交点缺失、歧义或退化，必须拒绝。");
            return result;
        }
        result.afterIntersection = inParent(candidate, after.modelPoint);
        result.afterErrorMm = (vector(result.afterIntersection) - vector(request.realWeld)).norm();
        result.valid = std::isfinite(result.beforeErrorMm) && std::isfinite(result.afterErrorMm);
        result.message = QStringLiteral("同一曝光平面与同一 CAD 焊缝边已分别重求交。");
        return result;
    } catch (const Standard_Failure &failure) {
        result.message = QStringLiteral("OCCT 求交异常：%1").arg(QString::fromUtf8(failure.GetMessageString()));
    } catch (const std::exception &failure) {
        result.message = QStringLiteral("候选几何评价异常：%1").arg(QString::fromUtf8(failure.what()));
    } catch (...) {
        result.message = QStringLiteral("候选几何评价发生未知异常。");
    }
    result.valid = false;
    result.status = OnlineSeamGeometryStatus::DegenerateIntersection;
    return result;
}

void COnlineSeamCandidateGeometry::clear()
{
    m_impl = std::make_unique<Impl>();
}

} // namespace i2r_rlc::digital_twin