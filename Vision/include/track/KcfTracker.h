/*******************************************************************************
 *       ______   ______   _______         __                  __
 *      /      | /      \ /       \       /  |                /  |
 *      $$$$$$/ /$$$$$$  |$$$$$$$  |      $$ |        ______  $$ |____
 *        $$ |  $$____$$ |$$ |__$$ |      $$ |       /      \ $$      \
 *        $$ |   /    $$/ $$    $$<       $$ |       $$$$$$  |$$$$$$$  |
 *        $$ |  /$$$$$$/  $$$$$$$  |      $$ |       /    $$ |$$ |  $$ |
 *       _$$ |_ $$ |_____ $$ |  $$ |      $$ |_____ /$$$$$$$ |$$ |__$$ |
 *      / $$   |$$       |$$ |  $$ |      $$       |$$    $$ |$$    $$/
 *      $$$$$$/ $$$$$$$$/ $$/   $$/       $$$$$$$$/  $$$$$$$/ $$$$$$$/
 *
 * @copyright   Copyright (c) 2024, 华南理工大学 I2R 实验室, All rights reserved.
 *
 * @file        KcfTracker.h
 * @brief       KCF 目标跟踪器
 * @details     以 OpenCV-only 方式迁移旧 KCFTracker 的 init/update 接口和核心跟踪语义。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 10 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260610      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260610      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#ifndef I2R_RLC_VISION_TRACK_CKCF_TRACKER_H
#define I2R_RLC_VISION_TRACK_CKCF_TRACKER_H

#include <deque>
#include <memory>

#include <opencv2/core.hpp>

#include "track/FilterKalman.h"
#include "track/Tracker.h"

namespace i2r_rlc::vision {

/**
 * @brief KCF 特征组合模式。
 *
 * 使用枚举替代裸整数，避免调用侧依赖魔法值。
 */
enum class KcfFeatureMode
{
    /// @brief 标准单模板 KCF。
    Standard = 0,
    /// @brief HOG 与灰度特征融合的 KCF。
    FusedHogGray = 1,
    /// @brief 旧工程 MKCF 多模板相关滤波路径。
    Mkcf = 2
};

/** @brief 在线焊缝位置融合策略；默认保留响应自适应 Kalman。 */
enum class KcfFusionMode
{
    AutomaticKalman = 0,
    ManualWeight = 1,
    KcfOnly = 2
};

/** @brief 跟踪线程按帧接收的融合配置；仿真权重范围为 [0, 1]。 */
struct KcfFusionOptions
{
    KcfFusionMode mode = KcfFusionMode::AutomaticKalman;
    double simulationWeight = 0.5;
};

/**
 * @brief 单次 update 的只读诊断快照；坐标为原图 ROI 左上角，单位 pixel。
 *
 * 标记决定字段是否可用；未融合时 gain/noise 不代表实际权重。
 */
struct CKcfUpdateDiagnostic {
    KcfFusionMode fusionMode = KcfFusionMode::AutomaticKalman;
    double simulationWeight = 0.5;
    bool hasResponse = false;
    bool completed = false;
    bool kalmanSeeded = false;
    bool fusionApplied = false;
    bool correlationJumpRejected = false;
    bool fusionJumpRejected = false;
    /// @brief 正式在线的独立搜索及延迟训练状态；普通调用保持关闭。
    bool onlineObservationMode = false;
    bool trainingDeferred = false;
    bool trainingCommitted = false;
    /// @brief 调用方接纳当前观测，但数值优化未生成可提交模型。
    bool trainingRejected = false;
    /// @brief 本帧响应实际使用的成功提交模型；0 表示无 MKCF 模型。
    int responseTemplateCount = 0;
    /// @brief 本帧响应的 HOG/灰度联合权重；不代表新帧训练后的权重。
    double responseHogWeight = 0.0;
    double responseGrayWeight = 0.0;
    /// @brief 式3-29响应图统计，不是目标身份或独立测量资格。
    double responsePsr = 0.0;
    /// @brief 本帧延迟训练采用仿真外观，不能记作实拍模板更新。
    bool trainingFromSimulation = false;
    double responsePeak = 0.0;
    double normalizedPeak = 0.0;
    double observationNoise = 0.0;
    double gainX = 0.0;
    double gainY = 0.0;
    cv::Rect2f responseRoi;
    cv::Rect2f beforeFusionRoi;
    cv::Rect2f finalRoi;
};

/**
 * @brief Kernelized Correlation Filter 跟踪器。
 *
 * 构造函数、`init` 和 `update` 保持原工程 KCFTracker 的调用方式。所有坐标单位均为 pixel。
 */
class CKcfTracker final : public CTracker {
public:
    /**
     * @brief 构造 KCF/MKCF 跟踪器。
     * @param hog 是否启用 FHOG 特征；关闭时使用灰度特征。
     * @param fixedWindow 是否使用固定模板窗口，保留旧工程参数语义。
     * @param multiscale 是否启用尺度搜索，保留旧工程参数语义。
     * @param lab 是否启用 Lab 颜色特征，当前 OpenCV-only 迁移仅保留兼容入口。
     * @param featureMode 特征组合和训练模式。
     * @param numTmpl MKCF 模板数量，单位为个；非 MKCF 模式会退回默认值。
     */
    CKcfTracker(
        bool hog = true,
        bool fixedWindow = true,
        bool multiscale = true,
        bool lab = true,
        KcfFeatureMode featureMode = KcfFeatureMode::Standard,
        int numTmpl = 2);

    /**
     * @brief 用首帧 ROI 初始化跟踪器。
     * @param roi 初始目标框，坐标和尺寸单位均为 pixel。
     * @param image 首帧图像；函数内部只保存所需模板，不接管外部 Mat 生命周期。
     */
    void init(const cv::Rect &roi, cv::Mat image) override;

    /**
     * @brief 在新图像上执行一次跟踪更新。
     * @param image 当前帧图像；调用线程需保证图像在调用期间有效。
     * @return 预测后的目标框，坐标和尺寸单位均为 pixel。
     */
    cv::Rect update(cv::Mat image) override;

    /**
     * @brief 重置 Kalman 平滑状态。
     *
     * 调用后下一次 update 会重新以当前响应初始化位置观测历史。
     */
    void initKF();

    /**
     * @brief 在跟踪线程热更新融合策略，不重建 KCF 模板。
     * @param options 非法模式回退自动 Kalman；非有限权重回退 0.5，其余限幅至 [0, 1]。
     * 策略或权重变化时清除滤波历史；重复提交同一配置不影响滤波连续性。
     * 调用方须与 init/update 串行执行。
     */
    void setFusionOptions(const KcfFusionOptions &options);

    /**
     * @brief 正式在线保留原始响应 ROI 作下帧搜索，并延迟模板训练至原图验收后。
     * @details 默认关闭。仍计算既有 Kalman/手动融合增益与融合 ROI 诊断；
     * 不把仿真融合位置写回独立观测的搜索或训练状态。调用方须串行提交训练。
     */
    void setOnlineObservationMode(bool enabled);

    /**
     * @brief 接受或放弃最近一帧待训练特征；重复提交无效果，不重新执行相关检测。
     * @details 只有调用方通过原图/方向校核的帧才可 accepted=true；最多保留一帧，
     * 后续 update、init 或模式切换会放弃未提交内容，拒绝帧不改变模板。
     */
    void commitOnlineObservationTraining(bool accepted);

    /**
     * @brief 放弃异常实拍待训练特征，以有效仿真外观替代检测模板（论文3-47）。
     * @details 只更新外观/系数，不将仿真ROI作为位置观测；无有效仿真时保持旧模型。
     * 与update串行调用，数值成功后才提交，来源写入诊断。
     */
    void commitOnlineSimulationTraining();

    /**
     * @brief 用同帧原图已校核的 ROI 修正搜索与待训练位置，保留原始相关响应诊断。
     * @details 不重置或训练模型，不接纳仿真坐标；只能在 update 后、训练提交前串行调用。
     */
    void correctOnlineObservationRoi(const cv::Rect2f &roi);

    /**
     * @brief 设置 MKCF 使用的仿真模板图像与 ROI。
     * @param image 仿真模板图像；函数内部拷贝，调用方可释放原图。
     * @param roi 模板 ROI，坐标和尺寸单位均为 pixel。
     */
    void setSimulatedTemplate(const cv::Mat &image, const cv::Rect &roi);

    /// @brief 清除 MKCF 仿真模板，2T/3T 路径会退回单模板训练。
    void clearSimulatedTemplate();

    /**
     * @brief 判断当前是否存在有效仿真模板。
     * @return `true` 表示已设置非空仿真图像和有效 ROI。
     */
    bool hasSimulatedTemplate() const;

    /// @brief 返回最近一次 update 的阶段快照；仅可在跟踪线程中读取。
    const CKcfUpdateDiagnostic &lastUpdateDiagnostic() const;

    /// @brief KCF 模板更新插值因子，取值通常为 `[0, 1]`。
    float m_interp_factor = 0.012F;
    /// @brief Gaussian kernel 带宽。
    float m_sigma = 0.6F;
    /// @brief 相关滤波岭回归正则项。
    float m_lambda = 0.0001F;
    /// @brief FHOG cell 尺寸，单位 pixel。
    int m_cell_size = 4;
    /// @brief 旧工程二级 cell 尺寸缓存，单位 pixel。
    int m_cell_sizeQ = 16;
    /// @brief 搜索窗口相对 ROI 的 padding 比例。
    float m_padding = 1.5F;
    /// @brief Gaussian peak 的标准差比例。
    float m_output_sigma_factor = 0.125F;
    /// @brief 模板归一化目标尺寸，单位 pixel。
    int m_template_size = 96;
    /// @brief 多尺度搜索步长，1.0 表示不调整尺度。
    float m_scale_step = 1.0F;
    /// @brief 多尺度响应权重。
    float m_scale_weight = 0.95F;
    /// @brief 旧工程模板修正开关。
    bool m_modify_flag = false;
    /// @brief 响应矩阵累计计数，保留旧工程调试状态。
    int m_responseMatCnt = 0;

    /// @brief Kalman 是否已用有效观测初始化。
    bool m_Kal_init = false;
    /// @brief Kalman 平滑后的 x 坐标，单位 pixel。
    double m_Kalman_status_x = 0.0;
    /// @brief Kalman 平滑后的 y 坐标，单位 pixel。
    double m_Kalman_status_y = 0.0;
    /// @brief 响应历史累计窗口长度，单位帧。
    int m_accum_count = 10;
    /// @brief 最近 x 位移或位置观测历史，单位 pixel。
    std::deque<double> m_accum_x;
    /// @brief 最近 y 位移或位置观测历史，单位 pixel。
    std::deque<double> m_accum_y;

    /// @brief MKCF 模板数量，单位为个。
    int m_numTmpl = 2;

protected:
    /**
     * @brief 在特征图上用当前模型检测目标偏移。
     * @param model 频域模型或 alpha 响应。
     * @param features 当前帧候选区域特征。
     * @param peakValue 输出最大响应值。
     * @return 相对搜索窗口中心的亚像素位移，单位 pixel。
     */
    cv::Point2f detect(const cv::Mat &model, const cv::Mat &features, float &peakValue);

    /**
     * @brief 使用指定模板和 alpha 响应检测目标偏移。
     * @param model 单模板特征模型。
     * @param alphaF 频域 alpha 响应。
     * @param features 当前帧候选区域特征。
     * @param peakValue 输出最大响应值。
     * @return 相对搜索窗口中心的亚像素位移，单位 cell。
     */
    cv::Point2f detectWithModelAlpha(
        const cv::Mat &model,
        const cv::Mat &alphaF,
        const cv::Mat &features,
        float &peakValue);

    /**
     * @brief 执行 MKCF 多模板检测。
     * @param image 当前帧图像。
     * @param peakResponse 输出融合后的响应峰值。
     * @return 相对当前 ROI 的位移，单位 pixel。
     */
    cv::Point2f detectMKCF(const cv::Mat &image, float &peakResponse);

    /**
     * @brief 用当前特征训练或更新单模板 KCF。
     * @param features 当前帧模板特征。
     * @param trainInterpFactor 模板更新插值因子，取值通常为 `[0, 1]`。
     */
    void train(const cv::Mat &features, float trainInterpFactor);

    /**
     * @brief 更新 MKCF 内部的标准 KCF 参考模型。
     * @param features 当前帧标准 KCF 特征。
     * @param trainInterpFactor 模板更新插值因子，取值通常为 `[0, 1]`。
     */
    void trainStandardReference(const cv::Mat &features, float trainInterpFactor);

    /**
     * @brief 根据当前 MKCF 模式训练多模板模型。
     * @param image 当前帧图像。
     * @param trainInterpFactor 模板更新插值因子，取值通常为 `[0, 1]`。
     * @return true 表示优化成功并原子提交模型；false 保留上次可用模型。
     */
    bool trainMKCF(const cv::Mat &image, float trainInterpFactor);

    /**
     * @brief 训练双模板 MKCF。
     * @param image 当前帧图像。
     * @param trainInterpFactor 模板更新插值因子。
     * @param mu 固定的跨模板耦合权重。
     * @param muInc 兼容旧调用签名，当前不改变优化目标。
     * @return true 表示优化成功并提交模型；false 保留上次可用模型。
     */
    bool train2TMKCF(const cv::Mat &image, float trainInterpFactor, double mu, double muInc);

    /**
     * @brief 训练三模板 MKCF。
     * @param image 当前帧图像。
     * @param trainInterpFactor 模板更新插值因子。
     * @param mu 固定的跨模板耦合权重。
     * @param muInc 兼容旧调用签名，当前不改变优化目标。
     * @return true 表示优化成功并提交模型；false 保留上次可用模型。
     */
    bool train3TMKCF(const cv::Mat &image, float trainInterpFactor, double mu, double muInc);

    /**
     * @brief 计算两个同尺寸特征图的 Gaussian kernel 相关。
     * @param left 左特征图。
     * @param right 右特征图。
     * @return 频域 kernel 响应。
     */
    cv::Mat gaussianCorrelation(const cv::Mat &left, const cv::Mat &right) const;

    /**
     * @brief 按指定输出尺寸和带宽计算 Gaussian kernel 相关。
     * @param left 左特征图。
     * @param right 右特征图。
     * @param height 输出响应高度，单位 pixel/cell。
     * @param width 输出响应宽度，单位 pixel/cell。
     * @param sigma Gaussian kernel 带宽。
     * @return 频域 kernel 响应。
     */
    cv::Mat gaussianCorrelation(
        const cv::Mat &left,
        const cv::Mat &right,
        int height,
        int width,
        float sigma) const;

    /**
     * @brief 创建训练用二维 Gaussian peak。
     * @param sizeY 高度，单位 cell 或 pixel。
     * @param sizeX 宽度，单位 cell 或 pixel。
     * @return 频域 Gaussian peak。
     */
    cv::Mat createGaussianPeak(int sizeY, int sizeX) const;

    /**
     * @brief 从当前 ROI 提取 KCF 特征。
     * @param image 输入图像。
     * @param initHann 是否同步初始化 Hann 窗。
     * @param scaleAdjust 尺度调整比例。
     * @return 已乘 Hann 窗的特征图。
     */
    cv::Mat getFeatures(const cv::Mat &image, bool initHann, float scaleAdjust = 1.0F);

    /**
     * @brief 从指定 ROI 提取 MKCF 主/辅特征。
     * @param image 输入图像。
     * @param roi 候选 ROI，单位 pixel。
     * @param scaleAdjust 尺度调整比例。
     * @param primaryFeature 输出主特征。
     * @param secondaryFeature 输出辅特征。
     */
    void getFeatures(
        const cv::Mat &image,
        const cv::Rect_<float> &roi,
        float scaleAdjust,
        cv::Mat &primaryFeature,
        cv::Mat &secondaryFeature);

    /**
     * @brief 提取 HOG 与灰度融合特征。
     * @param image 输入图像。
     * @param scaleAdjust 尺度调整比例。
     * @return 融合后的特征图。
     */
    cv::Mat fusedFeat(const cv::Mat &image, float scaleAdjust);

    /**
     * @brief 拼接 HOG 和灰度特征通道。
     * @param hogFeature HOG/FHOG 特征图。
     * @param grayFeature 灰度特征图。
     * @return 通道拼接后的特征图。
     */
    cv::Mat concatFeature(const cv::Mat &hogFeature, const cv::Mat &grayFeature) const;

    /// @brief 按当前模板尺寸创建 KCF Hann 窗。
    void createHanningMats();

    /**
     * @brief 创建指定尺寸的统一 Hann 窗。
     * @param height 高度，单位 cell 或 pixel。
     * @param width 宽度，单位 cell 或 pixel。
     */
    void createHannUnified(int height, int width);

    /**
     * @brief 根据相邻三个响应值估计亚像素峰值偏移。
     * @param left 左邻响应。
     * @param center 中心响应。
     * @param right 右邻响应。
     * @return 相对中心点的偏移，单位 pixel/cell。
     */
    float subPixelPeak(float left, float center, float right) const;

private:
    /**
     * @brief 将图像转换为 KCF 训练使用的浮点归一化范围。
     * @param image 输入输出图像。
     */
    void normalizeImage(cv::Mat &image) const;

    /**
     * @brief 将当前 ROI 限制在图像范围内。
     * @param image 当前帧图像，用于提供边界尺寸，单位 pixel。
     */
    void clampRoiToImage(const cv::Mat &image);

    /**
     * @brief 根据初始 ROI 初始化模板几何缓存。
     * @param roi 初始 ROI，坐标和尺寸单位均为 pixel。
     */
    void initializeTemplateGeometryFromRoi(const cv::Rect_<float> &roi);

    /**
     * @brief 按 ROI 和尺度截取模板图像。
     * @param image 输入图像。
     * @param roi 候选 ROI，单位 pixel。
     * @param scaleAdjust 尺度调整比例。
     * @return 截取并按模板尺寸归一化后的图像块。
     */
    cv::Mat extractPatchImage(const cv::Mat &image, const cv::Rect_<float> &roi, float scaleAdjust) const;

    /**
     * @brief 按 ROI 和尺度直接提取模板特征。
     * @param image 输入图像。
     * @param roi 候选 ROI，单位 pixel。
     * @param scaleAdjust 尺度调整比例。
     * @return 已归一化的特征图。
     */
    cv::Mat extractPatchFeature(const cv::Mat &image, const cv::Rect_<float> &roi, float scaleAdjust) const;

    /**
     * @brief 根据响应峰值映射 Kalman 观测噪声。
     * @param peakValue 当前相关响应峰值。
     * @return 归一化观测噪声系数，范围由 `m_cRMinimum/m_cRMaximum` 限制。
     */
    double normalizedObservationNoise(float peakValue);

    /**
     * @brief 计算双模板训练所需的 kernel 缓存。
     * @param currentPrimary 当前帧主特征。
     * @param currentSecondary 当前帧辅特征。
     * @param referencePrimary 参考模板主特征。
     * @param referenceSecondary 参考模板辅特征。
     * @param height kernel 高度。
     * @param width kernel 宽度。
     * @param primarySigma 主特征 Gaussian kernel 带宽。
     * @param secondarySigma 辅特征 Gaussian kernel 带宽。
     */
    void computeKernels(
        const cv::Mat &currentPrimary,
        const cv::Mat &currentSecondary,
        const cv::Mat &referencePrimary,
        const cv::Mat &referenceSecondary,
        int height,
        int width,
        float primarySigma,
        float secondarySigma);

    /**
     * @brief 计算三模板训练所需的两两 kernel 缓存。
     * @param template1Primary 第一模板主特征。
     * @param template1Secondary 第一模板辅特征。
     * @param template2Primary 第二模板主特征。
     * @param template2Secondary 第二模板辅特征。
     * @param template3Primary 第三模板主特征。
     * @param template3Secondary 第三模板辅特征。
     * @param height kernel 高度。
     * @param width kernel 宽度。
     * @param primarySigma 主特征 Gaussian kernel 带宽。
     * @param secondarySigma 辅特征 Gaussian kernel 带宽。
     */
    void computeKernels(
        const cv::Mat &template1Primary,
        const cv::Mat &template1Secondary,
        const cv::Mat &template2Primary,
        const cv::Mat &template2Secondary,
        const cv::Mat &template3Primary,
        const cv::Mat &template3Secondary,
        int height,
        int width,
        float primarySigma,
        float secondarySigma);

    KcfFusionOptions m_fusionOptions;
    CKcfUpdateDiagnostic m_updateDiagnostic;
    bool m_onlineObservationMode = false;
    cv::Mat m_pendingOnlineFeatures;
    cv::Mat m_pendingOnlineSecondaryFeatures;
    cv::Mat m_pendingOnlineImage;
    cv::Rect2f m_pendingOnlineRoi;
    cv::Mat m_alphaF;
    cv::Mat m_prob;
    /// @brief MKCF 求解使用零位移原点的标签谱；检测响应仍以图像中心表示零位移。
    cv::Mat m_mkcfProb;
    cv::Mat m_tmpl;
    cv::Mat m_hann;
    cv::Mat m_hann2d;
    cv::Mat m_hannCol;
    cv::Size m_tmplSize;
    float m_scale = 1.0F;
    bool m_hogFeatures = true;
    bool m_labFeatures = false;
    int m_imageCount = 0;
    KcfFeatureMode m_featureMode = KcfFeatureMode::Standard;
    /// @brief 与当前 alpha/检测基底一起提交的真实模板数，供响应来源审计。
    int m_trainedTemplateCount = 0;
    int m_sizePatch[3] = {0, 0, 0};
    float m_peakValueMaximum = 1.0F;
    float m_peakValueMinimum = 0.85F;
    std::unique_ptr<CFilterKalman> m_filter;

    cv::Mat m_standardReferenceAlphaF;
    cv::Mat m_standardReferenceTmpl;

    float m_upsilon = 1.0F;
    float m_kernelCoeffPrimary = 0.9F;
    float m_kernelCoeffSecondary = 0.1F;
    float m_sigmaPrimary = 0.6F;
    float m_sigmaSecondary = 0.2F;
    /// @brief 与 m_alphaF 同次训练的递推检测模板；历史保存上一已接纳基底参与联合训练。
    cv::Mat m_detectionTmplPrimary;
    cv::Mat m_detectionTmplSecondary;
    cv::Mat m_histTmplPrimary;
    cv::Mat m_histTmplSecondary;
    cv::Mat m_imageSim;
    cv::Rect m_imageSimROI;

    cv::Mat m_kernel11Primary;
    cv::Mat m_kernel11Secondary;
    cv::Mat m_kernel22Primary;
    cv::Mat m_kernel22Secondary;
    cv::Mat m_kernel33Primary;
    cv::Mat m_kernel33Secondary;
    cv::Mat m_kernel12Primary;
    cv::Mat m_kernel12Secondary;
    cv::Mat m_kernel21Primary;
    cv::Mat m_kernel21Secondary;
    cv::Mat m_kernel13Primary;
    cv::Mat m_kernel13Secondary;
    cv::Mat m_kernel23Primary;
    cv::Mat m_kernel23Secondary;
    cv::Mat m_kernel31Primary;
    cv::Mat m_kernel31Secondary;
    cv::Mat m_kernel32Primary;
    cv::Mat m_kernel32Secondary;
};

} // namespace i2r_rlc::vision

#endif // I2R_RLC_VISION_TRACK_CKCF_TRACKER_H
