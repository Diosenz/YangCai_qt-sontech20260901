/** @file OnlineSeamCandidateGeometry.h
 * @brief 在线工件候选的只读 CAD 焊缝求交，不修改 RobotSim 场景。
 */
#ifndef I2R_RLC_DIGITAL_TWIN_ONLINE_SEAM_CANDIDATE_GEOMETRY_H
#define I2R_RLC_DIGITAL_TWIN_ONLINE_SEAM_CANDIDATE_GEOMETRY_H

#include <memory>
#include <vector>

#include <QString>

#include "LocalizationTypes.h"

namespace i2r_rlc::digital_twin {

/** @brief 固定曝光时刻、工件父级坐标系下的归一激光平面；n dot p + d = 0。 */
struct OnlineSeamLaserPlane {
    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
    double dMm = 0.0;
};

/** @brief 显式只读场景文件与本次连接的对象身份；不能由工件名称猜测模型。 */
struct OnlineSeamSceneBinding {
    QString sceneFile;
    int workpieceId = -1;
    int parentId = -1;
    int sensorId = -1;
};

/** @brief 无法确认几何或分支时返回未验证，调用方须保留原有安全门禁。 */
enum class OnlineSeamGeometryStatus {
    Unavailable,
    InvalidBinding,
    InvalidInput,
    ReferenceMismatch,
    NoIntersection,
    AmbiguousIntersection,
    DegenerateIntersection,
    Succeeded
};

/** @brief 加载结果；仅 loaded=true 表示身份及模型可供候选评价。 */
struct OnlineSeamGeometryLoadResult {
    bool loaded = false;
    QString modelFile;
    std::vector<int> seamEdgeIds;
    QString message;
};

/**
 * @brief 同一曝光的评价输入，全部点/矩阵均为 mm 和工件父级参考。
 * @details correction 左乘 parentFromWorkpiece。currentVirtualWeld 只核对当前
 * 模型求交一致性，不是固定材料点，也不参与选择最贴近实测的候选分支。
 */
struct OnlineSeamGeometryRequest {
    OnlineSeamLaserPlane parentLaserPlane;
    localization::RigidTransform parentFromWorkpiece;
    localization::RigidTransform correction;
    localization::Point3D_mm currentVirtualWeld;
    localization::Point3D_mm realWeld;
    double referenceToleranceMm = 0.0;
    /// @brief 已确认的 CAD 边号；-1 仅允许全部配置边合计唯一交点，不按实测挑边。
    int seamEdgeId = -1;
};

/** @brief 真正重新求交所得的前后误差；valid=false 时不得消费默认数值验收。 */
struct OnlineSeamGeometryResult {
    bool valid = false;
    /// @brief 当前模型唯一交点已复现服务参考；true 时候选失败必须否决，不能回退旧点对。
    bool referenceVerified = false;
    OnlineSeamGeometryStatus status = OnlineSeamGeometryStatus::Unavailable;
    int seamEdgeId = -1;
    localization::Point3D_mm beforeIntersection;
    localization::Point3D_mm afterIntersection;
    double referenceDifferenceMm = 0.0;
    double beforeErrorMm = 0.0;
    double afterErrorMm = 0.0;
    QString message;
};

/**
 * @brief 独占 OCCT 模型缓存的纯几何评价器，无设备、通信或 QWidget 访问。
 * @details 生命周期及全部调用须由同一 worker 线程串行拥有；加载/求交均不可放
 * UI 线程。多交点、共面和身份不符不自动挑选分支。禁止用 SET_POSE 试探候选。
 */
class COnlineSeamCandidateGeometry final {
public:
    /** @brief 仅初始化轻量缓存；文件读取由 load 显式完成。 */
    COnlineSeamCandidateGeometry();
    /** @brief 在所属线程释放缓存的 CAD 对象。 */
    ~COnlineSeamCandidateGeometry();
    COnlineSeamCandidateGeometry(const COnlineSeamCandidateGeometry &) = delete;
    COnlineSeamCandidateGeometry &operator=(const COnlineSeamCandidateGeometry &) = delete;

    /** @brief 只读解析场景中指定工件、父级和传感器，加载实际 STEP 与焊缝边。 */
    OnlineSeamGeometryLoadResult load(const OnlineSeamSceneBinding &binding);
    /** @brief 固定父级激光平面，分别与当前/候选工件重新求交并核对当前仿真参考。 */
    OnlineSeamGeometryResult evaluate(const OnlineSeamGeometryRequest &request) const;
    /** @brief 场景、模型代次切换时显式释放旧几何，不删除文件。 */
    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace i2r_rlc::digital_twin

#endif // I2R_RLC_DIGITAL_TWIN_ONLINE_SEAM_CANDIDATE_GEOMETRY_H