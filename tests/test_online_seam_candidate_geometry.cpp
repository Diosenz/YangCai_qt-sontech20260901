/** @file test_online_seam_candidate_geometry.cpp
 * @brief 使用独立合成 STEP 验证候选重求交；不接设备或修改 RobotSim。
 */
#include "OnlineSeamCandidateGeometry.h"

#include <cmath>
#include <cstdio>
#include <limits>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Geom_BezierCurve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>

namespace {

using namespace i2r_rlc::digital_twin;
using Point = i2r_rlc::localization::Point3D_mm;
using Transform = i2r_rlc::localization::RigidTransform;

bool require(bool condition, const char *message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); }
    return condition;
}

bool almostEqual(double left, double right)
{
    return std::abs(left - right) < 1.0e-5;
}

/** @brief 生成单边模型及显式场景，不依赖现场特定工件文件。 */
bool writeFixture(const QString &directory, const TopoDS_Shape &shape)
{
    STEPControl_Writer writer;
    if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone
        || writer.Write((directory + QStringLiteral("/seam.step")).toLocal8Bit().constData())
            != IFSelect_RetDone) {
        return false;
    }
    QFile scene(directory + QStringLiteral("/scene.xml"));
    if (!scene.open(QIODevice::WriteOnly)) { return false; }
    const QByteArray xml = "<root><scene><OBJ_SHAPE><base_object>"
        "<id>8</id><parent_id>3</parent_id></base_object><shape><model>"
        "<path>seam.step</path></model></shape></OBJ_SHAPE>"
        "<SENSOR_LSL><base_object><id>13</id></base_object><lsl_camera>"
        "<workpiece>8</workpiece><weld_seam>1</weld_seam>"
        "</lsl_camera></SENSOR_LSL></scene></root>";
    return scene.write(xml) == xml.size();
}

OnlineSeamSceneBinding binding(const QString &directory)
{
    return {directory + QStringLiteral("/scene.xml"), 8, 3, 13};
}

OnlineSeamGeometryRequest request()
{
    OnlineSeamGeometryRequest result;
    result.parentLaserPlane = {1.0, 0.0, 0.0, 0.0};
    result.referenceToleranceMm = 0.01;
    result.currentVirtualWeld = {0.0, 0.0, 0.0};
    result.realWeld = {0.0, -2.0, 0.0};
    result.correction.rowMajor[3] = 2.0;
    return result;
}

bool testStraightReintersection()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    const auto shape = BRepBuilderAPI_MakeEdge(
        gp_Pnt(-20.0, -20.0, 0.0), gp_Pnt(20.0, 20.0, 0.0)).Edge();
    if (!require(directory.isValid() && writeFixture(directory.path(), shape),
            "write straight fixture")) { return false; }
    COnlineSeamCandidateGeometry geometry;
    if (!require(geometry.load(binding(directory.path())).loaded,
            "explicit scene loads synthetic CAD seam")) { return false; }
    const auto evaluated = geometry.evaluate(request());
    bool ok = require(evaluated.valid, "straight candidate has valid intersection");
    ok = require(almostEqual(evaluated.beforeErrorMm, 2.0)
        && almostEqual(evaluated.afterErrorMm, 0.0),
        "candidate reintersects fixed plane instead of transforming old point") && ok;
    ok = require(almostEqual(evaluated.afterIntersection.x_mm, 0.0)
        && almostEqual(evaluated.afterIntersection.y_mm, -2.0),
        "candidate intersection stays on exposure plane") && ok;

    auto wrongReference = request();
    wrongReference.currentVirtualWeld = {0.0, 4.0, 0.0};
    const auto mismatch = geometry.evaluate(wrongReference);
    ok = require(!mismatch.valid && !mismatch.referenceVerified
        && mismatch.status == OnlineSeamGeometryStatus::ReferenceMismatch,
        "wrong scene reference cannot become trusted CAD evidence") && ok;

    auto noIntersection = request();
    noIntersection.correction.rowMajor[3] = 50.0;
    const auto missing = geometry.evaluate(noIntersection);
    ok = require(!missing.valid && missing.referenceVerified
        && missing.status == OnlineSeamGeometryStatus::NoIntersection,
        "finite seam bounds reject missing candidate intersection") && ok;

    auto invalid = request();
    invalid.correction.rowMajor[0] = 2.0;
    const auto scaled = geometry.evaluate(invalid);
    ok = require(!scaled.valid && scaled.status == OnlineSeamGeometryStatus::InvalidInput,
        "nonrigid candidate is rejected") && ok;
    invalid = request();
    invalid.parentLaserPlane.nx = std::numeric_limits<double>::quiet_NaN();
    ok = require(!geometry.evaluate(invalid).valid, "nonfinite plane is rejected") && ok;

    // 任意父级旋转和平移：模型的激光截面与修正共同换系，误差不应改变。
    auto rotated = request();
    rotated.parentFromWorkpiece.rowMajor = {
        0.0, -1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 5.0,
        0.0, 0.0, 1.0, 6.0, 0.0, 0.0, 0.0, 1.0};
    rotated.parentLaserPlane = {0.0, 1.0, 0.0, -5.0};
    rotated.correction = Transform{};
    rotated.correction.rowMajor[7] = 2.0;
    rotated.currentVirtualWeld = {4.0, 5.0, 6.0};
    rotated.realWeld = {6.0, 5.0, 6.0};
    const auto changedFrame = geometry.evaluate(rotated);
    ok = require(changedFrame.valid && almostEqual(changedFrame.beforeErrorMm, 2.0)
        && almostEqual(changedFrame.afterErrorMm, 0.0),
        "general parent rotation and translation preserve geometry") && ok;

    auto wrongBinding = binding(directory.path());
    wrongBinding.parentId = 99;
    ok = require(!geometry.load(wrongBinding).loaded,
        "parent identity mismatch invalidates old cache") && ok;
    ok = require(!geometry.evaluate(request()).valid,
        "failed reload cannot retain previous model") && ok;
    return ok;
}

bool testGeneralCurve()
{
    TColgp_Array1OfPnt poles(1, 3);
    poles(1) = gp_Pnt(-10.0, 0.0, 0.0);
    poles(2) = gp_Pnt(0.0, 4.0, 0.0);
    poles(3) = gp_Pnt(10.0, 0.0, 0.0);
    Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
    const auto shape = BRepBuilderAPI_MakeEdge(curve).Edge();
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    if (!require(directory.isValid() && writeFixture(directory.path(), shape),
            "write noncircular curve fixture")) { return false; }
    COnlineSeamCandidateGeometry geometry;
    if (!require(geometry.load(binding(directory.path())).loaded,
            "load noncircular curve")) { return false; }
    auto input = request();
    input.currentVirtualWeld = {0.0, 2.0, 0.0};
    input.realWeld = {0.0, 1.92, 0.0};
    const auto result = geometry.evaluate(input);
    bool ok = require(result.valid && almostEqual(result.afterErrorMm, 0.0)
        && almostEqual(result.beforeErrorMm, 0.08),
        "noncircular curve uses CAD geometry without radius assumptions");
    input.parentLaserPlane = {0.0, 0.0, 1.0, 0.0};
    const auto coplanar = geometry.evaluate(input);
    ok = require(!coplanar.valid
        && coplanar.status == OnlineSeamGeometryStatus::DegenerateIntersection,
        "noncircular curve contained in exposure plane is degenerate") && ok;
    input.parentLaserPlane = {0.0, 0.0, 1.0, -1.0};
    const auto parallel = geometry.evaluate(input);
    ok = require(!parallel.valid
        && parallel.status == OnlineSeamGeometryStatus::NoIntersection,
        "parallel offset curve is missing rather than coplanar") && ok;
    return ok;
}

bool testDegenerateAndMultipleIntersections()
{
    QTemporaryDir directory;
    directory.setAutoRemove(false);
    const auto circle = BRepBuilderAPI_MakeEdge(gp_Circ(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 10.0)).Edge();
    if (!require(directory.isValid() && writeFixture(directory.path(), circle),
            "write multiple intersection fixture")) { return false; }
    COnlineSeamCandidateGeometry geometry;
    if (!require(geometry.load(binding(directory.path())).loaded, "load circle fixture")) { return false; }
    auto input = request();
    input.currentVirtualWeld = {0.0, 10.0, 0.0};
    input.realWeld = {0.0, -10.0, 0.0};
    const auto multiple = geometry.evaluate(input);
    bool ok = require(!multiple.valid
        && multiple.status == OnlineSeamGeometryStatus::AmbiguousIntersection,
        "real point cannot select a favorable branch among intersections");
    input.parentLaserPlane = {1.0, 0.0, 0.0, -10.0};
    input.currentVirtualWeld = {10.0, 0.0, 0.0};
    const auto tangent = geometry.evaluate(input);
    ok = require(!tangent.valid
        && tangent.status == OnlineSeamGeometryStatus::DegenerateIntersection,
        "tangent intersection does not establish material identity") && ok;
    input.parentLaserPlane = {0.0, 0.0, 1.0, 0.0};
    const auto coplanar = geometry.evaluate(input);
    ok = require(!coplanar.valid
        && coplanar.status == OnlineSeamGeometryStatus::DegenerateIntersection,
        "coplanar seam is not a unique laser intersection") && ok;
    geometry.clear();
    ok = require(!geometry.evaluate(request()).valid, "clear invalidates geometry") && ok;
    return ok;
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication application(argc, argv);
    const bool straight = testStraightReintersection();
    const bool curve = testGeneralCurve();
    const bool ambiguous = testDegenerateAndMultipleIntersections();
    if (!straight || !curve || !ambiguous) { return 1; }
    std::puts("online seam candidate geometry tests passed");
    return 0;
}