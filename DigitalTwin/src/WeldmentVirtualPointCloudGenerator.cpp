/**
 * @copyright Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file      WeldmentVirtualPointCloudGenerator.cpp
 * @brief     weldment 工件模型虚拟参考点云生成器实现。
 * @details   实现 .weldment 配置解析与 OpenCASCADE 面采样。
 * @version   1.0
 * @author    李力佳
 * @date      2026 年 8 月 8 日
 */

#include "WeldmentVirtualPointCloudGenerator.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <utility>
#include <unordered_set>

// OpenCASCADE core headers
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <Geom_Surface.hxx>
#include <gp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace i2r_rlc::digital_twin {

namespace {

// ===================== Enums =====================
// DOF 位掩码，与旧工程 WARPTYPE 定义保持一致（本生成器只解析配置，不参与求解）。
enum WarpType {
    WARPTYPE_NONE = 0,
    WARPTYPE_X = 1 << 0,
    WARPTYPE_Y = 1 << 1,
    WARPTYPE_Z = 1 << 2,
    WARPTYPE_RX = 1 << 3,
    WARPTYPE_RY = 1 << 4,
    WARPTYPE_RZ = 1 << 5
};

enum LocationFaceType {
    LOCATION_FACE_NONE,
    LOCATION_FACE_PLANE,
    LOCATION_FACE_CYLINDER,
    LOCATION_FACE_CONE
};

enum LocationElement {
    LOCATION_ELEMENT_NONE,
    LOCATION_ELEMENT_PLANE_ALIGN,
    LOCATION_ELEMENT_SHORT_SHAFT,
    LOCATION_ELEMENT_POINT_LINE_ALIGN
};

// ===================== Utility =====================
std::string trimString(const std::string& s)
{
    const std::size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return std::string();
    }
    const std::size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

void stringSeperate(const std::string& src, char delim, std::vector<std::string>& out)
{
    out.clear();
    std::stringstream ss(src);
    std::string item;
    while (std::getline(ss, item, delim)) {
        const std::string trimmed = trimString(item);
        if (!trimmed.empty()) {
            out.push_back(trimmed);
        }
    }
}

double deg2rad(double deg)
{
    return deg * 3.14159265358979323846 / 180.0;
}

// 与旧工程 string_rpy_to_matrix 对齐：x,y,z(mm) + rx,ry,rz(deg) -> gp_Trsf。
// 旋转组合顺序为 Rx -> Ry -> Rz，与 ComTest/virPoints.cpp 保持一致。
gp_Trsf stringRpyToTrsf(const std::string& rpyStr)
{
    std::vector<std::string> vals;
    stringSeperate(rpyStr, ',', vals);
    gp_Trsf trsf;
    if (vals.size() < 6) {
        return trsf;
    }

    const double x = std::stod(vals[0]);
    const double y = std::stod(vals[1]);
    const double z = std::stod(vals[2]);
    const double rx = deg2rad(std::stod(vals[3]));
    const double ry = deg2rad(std::stod(vals[4]));
    const double rz = deg2rad(std::stod(vals[5]));

    trsf.SetTranslation(gp_Vec(x, y, z));
    gp_Trsf rotX;
    gp_Trsf rotY;
    gp_Trsf rotZ;
    rotX.SetRotation(gp::OX(), rx);
    rotY.SetRotation(gp::OY(), ry);
    rotZ.SetRotation(gp::OZ(), rz);
    trsf.Multiply(rotX);
    trsf.Multiply(rotY);
    trsf.Multiply(rotZ);
    return trsf;
}

LocationFaceType str2LocationFaceType(const std::string& s)
{
    if (s == "plane") {
        return LOCATION_FACE_PLANE;
    }
    if (s == "cylinder") {
        return LOCATION_FACE_CYLINDER;
    }
    if (s == "cone") {
        return LOCATION_FACE_CONE;
    }
    return LOCATION_FACE_NONE;
}

LocationElement str2LocationElement(const std::string& s)
{
    if (s == "planePlaneAlign") {
        return LOCATION_ELEMENT_PLANE_ALIGN;
    }
    if (s == "shortShaft") {
        return LOCATION_ELEMENT_SHORT_SHAFT;
    }
    if (s == "pointLineAlign") {
        return LOCATION_ELEMENT_POINT_LINE_ALIGN;
    }
    return LOCATION_ELEMENT_NONE;
}

// ===================== Lightweight Config =====================
class Config {
public:
    struct ConfItem {
        std::string section;
        std::string parameter;
        std::string value;
    };

    bool readConf(std::istream& is)
    {
        m_confData.clear();
        std::string line;
        std::string currentSection;

        while (std::getline(is, line)) {
            const std::string trimLine = trimString(line);
            if (trimLine.empty() || trimLine[0] == '#') {
                continue;
            }
            if (trimLine.front() == '[' && trimLine.back() == ']') {
                currentSection = trimLine.substr(1, trimLine.size() - 2);
                continue;
            }
            const std::size_t colonPos = trimLine.find(':');
            if (colonPos == std::string::npos) {
                continue;
            }
            const std::string key = trimString(trimLine.substr(0, colonPos));
            const std::string val = trimString(trimLine.substr(colonPos + 1));
            m_confData.push_back({currentSection, key, val});
        }
        return true;
    }

    bool select(const std::string& section, const std::string& parameter, std::string& value) const
    {
        for (const ConfItem& item : m_confData) {
            if (item.section == section && item.parameter == parameter) {
                value = item.value;
                return true;
            }
        }
        return false;
    }

    bool select(const std::string& section, const std::string& parameter, int& value) const
    {
        std::string raw;
        if (!select(section, parameter, raw)) {
            return false;
        }
        std::stringstream ss(raw);
        ss >> value;
        return !ss.fail();
    }

private:
    std::vector<ConfItem> m_confData;
};

// ===================== Weldment Config =====================
struct WeldmentConfig {
    gp_Trsf fixPose;
    std::vector<int> seamList;
    std::unordered_set<int> positionFaceList;
    std::vector<std::pair<std::vector<int>, WarpType>> positionFaceDofList;
    std::vector<std::pair<LocationFaceType, LocationElement>> positionFaceElementList;
    bool valid = false;
};

bool parseWeldmentConfig(const std::string& pathname, WeldmentConfig& config)
{
    std::ifstream ifile(pathname, std::ios::in);
    if (!ifile.is_open()) {
        return false;
    }
    Config conf;
    if (!conf.readConf(ifile)) {
        return false;
    }
    ifile.close();

    std::string tmpstr;
    std::vector<std::string> tmplist;
    std::vector<std::string> tmplist2;

    // 1. 装夹位姿
    if (!conf.select("weldment_parameters", "fix_pose/mmdeg", tmpstr)) {
        return false;
    }
    config.fixPose = stringRpyToTrsf(tmpstr);

    // 2. 焊缝列表
    config.seamList.clear();
    if (conf.select("weldment_parameters", "seam_list", tmpstr)) {
        stringSeperate(tmpstr, ',', tmplist);
        for (const std::string& s : tmplist) {
            config.seamList.push_back(std::stoi(s));
        }
    }

    // 3. 定位段
    int segmentCount = 0;
    if (!conf.select("weldment_parameters", "position_segment_count", segmentCount)) {
        return false;
    }
    config.positionFaceList.clear();
    config.positionFaceDofList.clear();
    config.positionFaceElementList.clear();

    for (int i = 1; i <= segmentCount; ++i) {
        if (!conf.select(
                "weldment_parameters",
                "position_segment_" + std::to_string(i),
                tmpstr)) {
            return false;
        }
        tmplist.clear();
        stringSeperate(tmpstr, '-', tmplist);
        if (tmplist.size() != 4) {
            return false;
        }

        const LocationFaceType lft = str2LocationFaceType(tmplist[0]);
        if (lft == LOCATION_FACE_NONE) {
            return false;
        }
        const LocationElement le = str2LocationElement(tmplist[1]);
        if (le == LOCATION_ELEMENT_NONE) {
            return false;
        }
        config.positionFaceElementList.push_back({lft, le});

        std::pair<std::vector<int>, WarpType> ps;
        tmplist2.clear();
        stringSeperate(tmplist[2], ',', tmplist2);
        for (const std::string& s : tmplist2) {
            const int fid = std::stoi(s);
            ps.first.push_back(fid);
            config.positionFaceList.insert(fid);
        }

        tmplist2.clear();
        stringSeperate(tmplist[3], '|', tmplist2);
        ps.second = WARPTYPE_NONE;
        for (const std::string& s : tmplist2) {
            if (s == "x") {
                ps.second = WarpType(ps.second | WARPTYPE_X);
            } else if (s == "y") {
                ps.second = WarpType(ps.second | WARPTYPE_Y);
            } else if (s == "z") {
                ps.second = WarpType(ps.second | WARPTYPE_Z);
            } else if (s == "rx") {
                ps.second = WarpType(ps.second | WARPTYPE_RX);
            } else if (s == "ry") {
                ps.second = WarpType(ps.second | WARPTYPE_RY);
            } else if (s == "rz") {
                ps.second = WarpType(ps.second | WARPTYPE_RZ);
            }
        }
        config.positionFaceDofList.push_back(ps);
    }

    config.valid = true;
    return true;
}

// 按面 id 提取面，id 从 1 开始，遍历顺序与旧工程 TopExp_Explorer 索引一致。
TopoDS_Face getFaceById(const TopoDS_Shape& shape, int faceId)
{
    if (faceId < 1) {
        return TopoDS_Face();
    }
    TopExp_Explorer explorer;
    explorer.Init(shape, TopAbs_FACE);
    int currentId = 1;
    for (; explorer.More(); explorer.Next(), ++currentId) {
        if (currentId == faceId) {
            const TopoDS_Shape& cur = explorer.Current();
            if (cur.ShapeType() == TopAbs_FACE) {
                return static_cast<const TopoDS_Face&>(cur);
            }
            break;
        }
    }
    return TopoDS_Face();
}

// 在单个定位基准面上 UV 随机采样，逻辑与 stepICPforOffline 内层循环一致。
std::vector<localization::Point3D_mm> samplePointsOnFace(
    const TopoDS_Face& face,
    const gp_Trsf& globalTrsf,
    double sampleRate)
{
    std::vector<localization::Point3D_mm> result;
    if (face.IsNull()) {
        return result;
    }

    Standard_Real uMin = 0.0;
    Standard_Real uMax = 0.0;
    Standard_Real vMin = 0.0;
    Standard_Real vMax = 0.0;
    BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

    BRepTopAdaptor_FClass2d classifier(face, Precision::Confusion());

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    const double faceArea = props.Mass();
    int sampleCount = static_cast<int>(sampleRate * faceArea);
    if (sampleCount < 10) {
        sampleCount = 10;
    }

    TopLoc_Location faceLoc;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face, faceLoc);
    if (surface.IsNull()) {
        return result;
    }

    // 变换组合顺序与原工程一致：面局部变换 -> 全局装夹位姿。
    gp_Trsf totalTrsf = faceLoc.Transformation();
    totalTrsf.Multiply(globalTrsf);

    // 固定随机种子，保证结果可复现。
    std::mt19937 randomGen(12345);
    std::uniform_real_distribution<double> uDist(uMin, uMax);
    std::uniform_real_distribution<double> vDist(vMin, vMax);

    int valid = 0;
    while (valid < sampleCount) {
        const double u = uDist(randomGen);
        const double v = vDist(randomGen);
        if (classifier.Perform(gp_Pnt2d(u, v)) == TopAbs_OUT) {
            continue;
        }

        gp_Pnt pnt;
        gp_Vec dU;
        gp_Vec dV;
        surface->D1(u, v, pnt, dU, dV);
        pnt.Transform(totalTrsf);

        result.push_back({
            pnt.X(),
            pnt.Y(),
            pnt.Z()});
        ++valid;
    }
    return result;
}

} // namespace

localization::IcpDofMask toIcpDofMask(WarpType warpType)
{
    localization::IcpDofMask mask;
    mask.allowX = (warpType & WARPTYPE_X) != 0;
    mask.allowY = (warpType & WARPTYPE_Y) != 0;
    mask.allowZ = (warpType & WARPTYPE_Z) != 0;
    mask.allowRx = (warpType & WARPTYPE_RX) != 0;
    mask.allowRy = (warpType & WARPTYPE_RY) != 0;
    mask.allowRz = (warpType & WARPTYPE_RZ) != 0;
    return mask;
}

WeldmentVirtualPointCloudResult WeldmentVirtualPointCloudGenerator::generate(
    const std::string& weldmentPath,
    double sampleRate) const
{
    WeldmentVirtualPointCloudResult result;

    // ========== 1. 读取 STEP 模型 ==========
    // .weldment 文件本质是 STEP 几何 + 尾部配置文本，STEP 解析器会忽略尾部文本。
    STEPControl_Reader stepReader;
    if (stepReader.ReadFile(weldmentPath.c_str()) != IFSelect_RetDone) {
        result.ok = false;
        result.message = "failed to read STEP model: " + weldmentPath;
        return result;
    }
    stepReader.TransferRoots();
    const TopoDS_Shape modelShape = stepReader.OneShape();
    if (modelShape.IsNull()) {
        result.ok = false;
        result.message = "STEP model shape is null: " + weldmentPath;
        return result;
    }

    // ========== 2. 解析配置 ==========
    WeldmentConfig config;
    if (!parseWeldmentConfig(weldmentPath, config)) {
        result.ok = false;
        result.message = "failed to parse [weldment_parameters] from: " + weldmentPath;
        return result;
    }
    if (config.positionFaceDofList.empty()) {
        result.ok = false;
        result.message = "no positioning segment configured in: " + weldmentPath;
        return result;
    }

    // ========== 3. 按定位段采样 ==========
    std::vector<std::size_t> segmentCounts;
    for (const auto& segment : config.positionFaceDofList) {
        std::vector<localization::Point3D_mm> segmentPoints;
        std::size_t segmentPointCount = 0;
        for (const int faceId : segment.first) {
            const TopoDS_Face face = getFaceById(modelShape, faceId);
            if (face.IsNull()) {
                result.message += "face id " + std::to_string(faceId) + " not found; ";
                continue;
            }
            const std::vector<localization::Point3D_mm> facePoints =
                samplePointsOnFace(face, config.fixPose, sampleRate);
            segmentPoints.insert(
                segmentPoints.end(),
                facePoints.begin(),
                facePoints.end());
            segmentPointCount += facePoints.size();
        }
        result.points_mm.insert(
            result.points_mm.end(),
            segmentPoints.begin(),
            segmentPoints.end());
        result.segmentPoints_mm.push_back(std::move(segmentPoints));
        result.segmentDofMasks.push_back(toIcpDofMask(segment.second));
        segmentCounts.push_back(segmentPointCount);
    }

    if (result.points_mm.empty()) {
        result.ok = false;
        result.message = "no virtual points generated from: " + weldmentPath;
        return result;
    }

    result.ok = true;
    result.message = "generated " + std::to_string(result.points_mm.size())
        + " virtual points from " + std::to_string(config.positionFaceDofList.size())
        + " positioning segments";
    result.segmentPointCounts = std::move(segmentCounts);
    result.referenceFaceCount = config.positionFaceList.size();
    return result;
}

} // namespace i2r_rlc::digital_twin
