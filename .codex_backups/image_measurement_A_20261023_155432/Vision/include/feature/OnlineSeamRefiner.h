/**
 * @file OnlineSeamRefiner.h
 * @brief 在线焊缝粗跟踪后的原图几何校核，不使用仿真点约束输出。
 */
#ifndef I2R_RLC_VISION_FEATURE_ONLINE_SEAM_REFINER_H
#define I2R_RLC_VISION_FEATURE_ONLINE_SEAM_REFINER_H

#include <opencv2/core.hpp>

namespace i2r_rlc::vision {

/** @brief 几何无效与纵向语义不确定必须区别于粗跟踪失败。 */
enum class OnlineSeamRefinementStatus {
    InvalidInput,
    InsufficientSupport,
    AmbiguousStripe,
    InconsistentGeometry,
    UnstablePosition,
    AmbiguousHeight,
    Succeeded
};

/** @brief 台阶的纵向目标必须由业务明确选择，算法不隐式决定焊接语义。 */
enum class OnlineSeamHeightPolicy {
    RequireContinuous,
    GapCenter
};

/** @brief 所有尺寸均为原图像素；默认参数只覆盖局部向下凸折点。 */
struct OnlineSeamRefinementOptions {
    int searchHalfWidthPixel = 100;
    int searchHalfHeightPixel = 60;
    int minSideSupportColumns = 25;
    double minContrastGrayValue = 40.0;
    double minSupportRatio = 0.75;
    double maxFitRmsPixel = 1.8;
    double maxPositionUncertaintyPixel = 6.0;
    double maxHeightJumpPixel = 3.0;
    /// @brief GapCenter 只允许有限台阶，同时受搜索半高四分之一限制。
    double maxGapHeightPixel = 16.0;
    OnlineSeamHeightPolicy heightPolicy = OnlineSeamHeightPolicy::RequireContinuous;
    /// @brief 两峰相距至少该像素数才可能是不同条纹，不区分同一亮带肩部。
    int minPeakSeparationPixel = 8;
    /// @brief 扣背景后次峰达到主峰该比例，才视为竞争条纹。
    double secondaryPeakRatio = 0.8;
    /// @brief 有效列中双窄峰比例超过该值时，拒绝整体单条纹假设。
    double maxAmbiguousColumnRatio = 0.2;
};

/**
 * @brief 原图几何观测；horizontalValid 不允许直接替代完整二维 valid。
 *
 * point 仅在 valid 时可投影到三维。两侧 endpoint 是拟合线在分割 X 的
 * 端部估计。只有调用方明确选择 GapCenter 时，才用二者中部表示
 * 加工接缝的焊接目标高度；RequireContinuous 不对台阶作该解释。
 */
struct OnlineSeamRefinementResult {
    bool horizontalValid = false;
    bool valid = false;
    OnlineSeamRefinementStatus status = OnlineSeamRefinementStatus::InvalidInput;
    cv::Point2d point = cv::Point2d(-1.0, -1.0);
    cv::Point2d leftEndpoint = cv::Point2d(-1.0, -1.0);
    cv::Point2d rightEndpoint = cv::Point2d(-1.0, -1.0);
    /// @brief 当前原图双侧拟合的 dy/dx；仅 hasBranchDirections 且 valid 时可续接历史。
    bool hasBranchDirections = false;
    double leftSlope = 0.0;
    double rightSlope = 0.0;
    /// @brief 支撑和拟合残差生成的质量分数，非校准概率或物理精度。
    double confidence = 0.0;
    double supportRatio = 0.0;
    double fitRmsPixel = 0.0;
    /// @brief 近优分割代价平台宽度，非统计置信区间，单位 pixel。
    double xUncertaintyPixel = 0.0;
    /// @brief 右侧端部减左侧端部的高度差，图像 Y 向下为正，单位 pixel。
    double heightJumpPixel = 0.0;
    /// @brief 检测到两条接近等强窄条纹的有效列比例，范围 0~1。
    double ambiguousColumnRatio = 0.0;
    int leftSupportColumns = 0;
    int rightSupportColumns = 0;
};

/**
 * @brief 无状态、无设备和线程所有权的原图接缝精定位器。
 *
 * 只能在视觉工作线程执行。方法不修改输入、不保留图像，不训练 KCF。
 * 支撑不足或台阶高度策略要求拒绝时显式返回无效，调用方不得静默回退到
 * 粗 KCF 点作为经过校核的焊点。
 */
class COnlineSeamRefiner final {
public:
    /**
     * @brief 在 CV_8UC1 原图中独立估计条纹转折及其可见支撑。
     * @param gray8 同帧原图，禁止传入二值化或 Gabor 增强图。
     * @param coarsePoint KCF 提供的搜索中心，单位为原图像素。
     * @param options 局部搜索和拒绝门槛，不包含仿真目标位置。
     * @return 完整二维有效点或明确的几何拒绝原因。
     */
    static OnlineSeamRefinementResult refine(
        const cv::Mat &gray8,
        const cv::Point2d &coarsePoint,
        const OnlineSeamRefinementOptions &options);
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_FEATURE_ONLINE_SEAM_REFINER_H
