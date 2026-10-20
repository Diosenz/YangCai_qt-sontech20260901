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
 * @file        KcfTracker.cpp
 * @brief       KCF 目标跟踪器实现
 * @details     保持旧 KCFTracker 的 init/update 入口，使用 OpenCV 相关响应与 RMatrix Kalman。
 *
 * @version     1.1
 * @author      LT
 * @date        2026 年 6 月 6 日
 *
 * @par         修改日志
 * <table>
 * <tr><th>Version  <th>Author      <th>Date        <th>Description         </tr>
 * <tr><td>1.0      <td>LT          <td>260606      <td>创建文件            </tr>
 * <tr><td>1.1      <td>LT          <td>260606      <td>初步完成            </tr>
 * </table>
 *
 ******************************************************************************/
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <vector>

#include <cassert>
#include <opencv2/imgproc.hpp>

#include "track/KcfFftTools.h"
#include "track/KcfFhog.h"
#include "track/KcfLabData.h"
#include "track/KcfRectTools.h"
#include "track/KcfTracker.h"

namespace i2r_rlc::vision {

namespace {

/**
 * @brief 将浮点 ROI 转为 OpenCV 整数矩形。
 * @param rect 浮点 ROI，坐标单位为 pixel。
 * @return 四舍五入后的整数 ROI。
 */
cv::Rect toIntegerRect(const cv::Rect_<float> &rect)
{
    return cv::Rect(
        static_cast<int>(std::round(rect.x)),
        static_cast<int>(std::round(rect.y)),
        static_cast<int>(std::round(rect.width)),
        static_cast<int>(std::round(rect.height)));
}

/**
 * @brief 按 padding 从当前 ROI 推导搜索窗口。
 * @param roi 当前跟踪 ROI，单位 pixel。
 * @param padding 搜索窗口相对 ROI 扩张比例。
 * @return 扩张后的搜索窗口，单位 pixel。
 */
cv::Rect searchWindowFromRoi(const cv::Rect_<float> &roi, double padding)
{
    const double cx = roi.x + roi.width * 0.5;
    const double cy = roi.y + roi.height * 0.5;
    const int width = std::max(3, static_cast<int>(std::round(roi.width * (1.0 + padding))));
    const int height = std::max(3, static_cast<int>(std::round(roi.height * (1.0 + padding))));
    return cv::Rect(
        static_cast<int>(std::round(cx - width * 0.5)),
        static_cast<int>(std::round(cy - height * 0.5)),
        width,
        height);
}

/**
 * @brief 仅对复数矩阵实部叠加标量。
 * @param value 单通道实数矩阵或双通道复数矩阵。
 * @param scalar 要叠加到实部的数值。
 * @return 叠加后的矩阵副本。
 */
cv::Mat addRealScalar(const cv::Mat &value, float scalar)
{
    if (value.empty())
    {
        return value.clone();
    }
    if (value.channels() == 1)
    {
        return value + scalar;
    }

    std::vector<cv::Mat> channels;
    cv::split(value, channels);
    channels[0] += scalar;
    cv::Mat output;
    cv::merge(channels, output);
    return output;
}

/**
 * @brief 处理空矩阵兼容逻辑的复数矩阵加法。
 * @param left 左矩阵。
 * @param right 右矩阵。
 * @return 两个矩阵相加后的结果；空输入按零矩阵处理。
 */
cv::Mat complexAddition(const cv::Mat &left, const cv::Mat &right)
{
    if (left.empty())
    {
        return right.clone();
    }
    if (right.empty())
    {
        return left.clone();
    }
    return left + right;
}

/**
 * @brief 对复数矩阵所有通道乘以实数标量。
 * @param value 输入矩阵。
 * @param scalar 实数缩放系数。
 * @return 缩放后的矩阵。
 */
cv::Mat complexMultiplicationScalar(const cv::Mat &value, float scalar)
{
    return value * scalar;
}

/**
 * @brief 统计矩阵实部之和。
 * @param value 单通道实数矩阵或双通道复数矩阵。
 * @return 实部元素求和结果。
 */
double sumRealPart(const cv::Mat &value)
{
    if (value.empty())
    {
        return 0.0;
    }
    if (value.channels() == 1)
    {
        return cv::sum(value)[0];
    }
    std::vector<cv::Mat> planes;
    cv::split(value, planes);
    return planes.empty() ? 0.0 : cv::sum(planes[0])[0];
}

/**
 * @brief 计算 `abs2Mat` 与 kernel 实部逐点乘积之和。
 * @param abs2Mat 单通道平方幅值矩阵。
 * @param kernelMat 单通道或双通道 kernel 矩阵。
 * @return 累计乘积，供 MKCF 约束项计算使用。
 */
double sumAbs2MulRealK(const cv::Mat &abs2Mat, const cv::Mat &kernelMat)
{
    CV_Assert(abs2Mat.type() == CV_32F);
    CV_Assert(abs2Mat.size() == kernelMat.size());

    double sum = 0.0;
    if (kernelMat.channels() == 1)
    {
        for (int row = 0; row < abs2Mat.rows; ++row)
        {
            const float *absPtr = abs2Mat.ptr<float>(row);
            const float *kernelPtr = kernelMat.ptr<float>(row);
            for (int col = 0; col < abs2Mat.cols; ++col)
            {
                sum += static_cast<double>(absPtr[col]) * static_cast<double>(kernelPtr[col]);
            }
        }
        return sum;
    }

    for (int row = 0; row < abs2Mat.rows; ++row)
    {
        const float *absPtr = abs2Mat.ptr<float>(row);
        const cv::Vec2f *kernelPtr = kernelMat.ptr<cv::Vec2f>(row);
        for (int col = 0; col < abs2Mat.cols; ++col)
        {
            sum += static_cast<double>(absPtr[col]) * static_cast<double>(kernelPtr[col][0]);
        }
    }
    return sum;
}

/**
 * @brief 快速累加 MKCF 目标函数中的 A 项。
 * @param accumulator 输入输出累计值。
 * @param abs2Mat alpha 平方幅值矩阵。
 * @param kernelM 模板间 kernel 矩阵。
 * @param kernelP 模板间 kernel 矩阵。
 */
void addAFast(double &accumulator, const cv::Mat &abs2Mat, const cv::Mat &kernelM, const cv::Mat &kernelP)
{
    CV_Assert(abs2Mat.type() == CV_32F);
    CV_Assert(kernelM.type() == CV_32FC2 && kernelP.type() == CV_32FC2);
    CV_Assert(abs2Mat.size() == kernelM.size() && kernelM.size() == kernelP.size());

    double sum = 0.0;
    for (int row = 0; row < abs2Mat.rows; ++row)
    {
        const float *absPtr = abs2Mat.ptr<float>(row);
        const cv::Vec2f *mPtr = kernelM.ptr<cv::Vec2f>(row);
        const cv::Vec2f *pPtr = kernelP.ptr<cv::Vec2f>(row);
        for (int col = 0; col < abs2Mat.cols; ++col)
        {
            const float mRe = mPtr[col][0];
            const float mIm = mPtr[col][1];
            const float pRe = pPtr[col][0];
            const float pIm = pPtr[col][1];
            sum += static_cast<double>(absPtr[col]) * static_cast<double>(mRe * pRe - mIm * pIm);
        }
    }
    accumulator += sum;
}

/**
 * @brief 计算 `conj(alpha) * kernel * beta` 的实部和。
 * @param alpha 左侧频域 alpha。
 * @param kernel 中间 kernel 响应。
 * @param beta 右侧频域 alpha 或概率响应。
 * @return 实部累计值。
 */
double sumConjAMulKMulB(const cv::Mat &alpha, const cv::Mat &kernel, const cv::Mat &beta)
{
    CV_Assert(alpha.type() == CV_32FC2 && kernel.type() == CV_32FC2 && beta.type() == CV_32FC2);
    CV_Assert(alpha.size() == kernel.size() && kernel.size() == beta.size());

    double sum = 0.0;
    for (int row = 0; row < alpha.rows; ++row)
    {
        const cv::Vec2f *aPtr = alpha.ptr<cv::Vec2f>(row);
        const cv::Vec2f *kPtr = kernel.ptr<cv::Vec2f>(row);
        const cv::Vec2f *bPtr = beta.ptr<cv::Vec2f>(row);
        for (int col = 0; col < alpha.cols; ++col)
        {
            const float aRe = aPtr[col][0];
            const float aIm = aPtr[col][1];
            const float kRe = kPtr[col][0];
            const float kIm = kPtr[col][1];
            const float bRe = bPtr[col][0];
            const float bIm = bPtr[col][1];
            const float kbRe = kRe * bRe - kIm * bIm;
            const float kbIm = kRe * bIm + kIm * bRe;
            sum += static_cast<double>(aRe) * static_cast<double>(kbRe)
                + static_cast<double>(aIm) * static_cast<double>(kbIm);
        }
    }
    return sum;
}

/**
 * @brief 计算 `conj(alpha) * kernel * prob` 的实部和。
 * @param alpha 频域 alpha。
 * @param kernel kernel 响应。
 * @param prob Gaussian peak 频域响应。
 * @return 实部累计值。
 */
double sumConjAMulKMulProb(const cv::Mat &alpha, const cv::Mat &kernel, const cv::Mat &prob)
{
    return sumConjAMulKMulB(alpha, kernel, prob);
}

/**
 * @brief 计算复数矩阵逐元素平方幅值。
 * @param complexMat `CV_32FC2` 复数矩阵。
 * @param abs2Mat 输出单通道平方幅值矩阵。
 */
void computeAbs2(const cv::Mat &complexMat, cv::Mat &abs2Mat)
{
    CV_Assert(complexMat.type() == CV_32FC2);
    abs2Mat.create(complexMat.rows, complexMat.cols, CV_32F);
    for (int row = 0; row < complexMat.rows; ++row)
    {
        const cv::Vec2f *complexPtr = complexMat.ptr<cv::Vec2f>(row);
        float *absPtr = abs2Mat.ptr<float>(row);
        for (int col = 0; col < complexMat.cols; ++col)
        {
            const float re = complexPtr[col][0];
            const float im = complexPtr[col][1];
            absPtr[col] = re * re + im * im;
        }
    }
}

/**
 * @brief 计算观测历史均值。
 * @param values 观测历史序列。
 * @return 均值；空序列返回 0。
 */
double calcMean(const std::deque<double> &values)
{
    if (values.empty())
    {
        return 0.0;
    }
    double sum = 0.0;
    for (double value : values)
    {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

/**
 * @brief 计算观测历史样本标准差。
 * @param values 观测历史序列。
 * @return 样本标准差；样本不足时返回大值避免过度信任观测。
 */
double calcStdDev(const std::deque<double> &values)
{
    if (values.size() <= 1)
    {
        return 1e9;
    }
    const double mean = calcMean(values);
    double sumSquared = 0.0;
    for (double value : values)
    {
        const double diff = value - mean;
        sumSquared += diff * diff;
    }
    return std::sqrt(sumSquared / static_cast<double>(values.size() - 1));
}

/**
 * @brief 计算复数矩阵 Frobenius 范数平方。
 * @param complexMat `CV_32FC2` 复数矩阵。
 * @return 所有元素平方幅值之和。
 */
double calcFrobeniusNorm2(const cv::Mat &complexMat)
{
    CV_Assert(complexMat.type() == CV_32FC2);
    std::vector<cv::Mat> channels;
    cv::split(complexMat, channels);
    cv::Mat magnitude2 = channels[0].mul(channels[0]) + channels[1].mul(channels[1]);
    return static_cast<double>(cv::sum(magnitude2)[0]);
}

/**
 * @brief 计算两个复数矩阵差值的 Frobenius 范数平方。
 * @param left 左矩阵。
 * @param right 右矩阵。
 * @return `(left - right)` 的平方范数。
 */
double calcDiffFrobeniusNorm2(const cv::Mat &left, const cv::Mat &right)
{
    return calcFrobeniusNorm2(left - right);
}

} // namespace

CKcfTracker::CKcfTracker(
    bool hog,
    bool fixedWindow,
    bool multiscale,
    bool lab,
    KcfFeatureMode featureMode,
    int numTmpl)
{
    m_lambda = 0.0001F;
    m_padding = 1.5F;
    m_output_sigma_factor = 0.125F;
    m_modify_flag = false;
    m_featureMode = featureMode;
    m_numTmpl = featureMode == KcfFeatureMode::Mkcf ? numTmpl : 2;

    if (hog)
    {
        m_interp_factor = 0.012F;
        m_sigma = 0.6F;
        m_cell_size = 4;
        m_hogFeatures = true;
        if (lab)
        {
            m_interp_factor = 0.005F;
            m_sigma = 0.4F;
            m_output_sigma_factor = 0.1F;
            m_labFeatures = true;
        }
        else
        {
            m_labFeatures = false;
        }
    }
    else
    {
        m_interp_factor = 0.075F;
        m_sigma = 0.2F;
        m_cell_size = 1;
        m_hogFeatures = false;
        m_labFeatures = false;
    }

    m_cell_sizeQ = m_cell_size * m_cell_size;
    m_sigmaPrimary = m_sigma;
    m_sigmaSecondary = std::max(0.2F, m_sigma * 0.5F);
    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
    m_upsilon = 1.0F;
    if (multiscale)
    {
        m_template_size = 96;
        m_scale_step = 1.05F;
        m_scale_weight = 0.95F;
        (void)fixedWindow;
    }
    else if (fixedWindow)
    {
        m_template_size = 96;
        m_scale_step = 1.0F;
    }
    else
    {
        m_template_size = 1;
        m_scale_step = 1.0F;
    }
}

void CKcfTracker::init(const cv::Rect &roi, cv::Mat image)
{
    m_updateDiagnostic = {};
    m_modify_flag = false;
    m_imageCount = 1;
    m_roi = roi;
    normalizeImage(image);
    clampRoiToImage(image);

    if (m_featureMode == KcfFeatureMode::Standard || m_featureMode == KcfFeatureMode::Mkcf)
    {
        m_tmpl = getFeatures(image, true, 1.0F);
    }
    else if (m_featureMode == KcfFeatureMode::FusedHogGray)
    {
        m_tmpl = fusedFeat(image, 1.0F);
    }

    m_prob = createGaussianPeak(m_sizePatch[0], m_sizePatch[1]);
    m_alphaF = cv::Mat::zeros(m_sizePatch[0], m_sizePatch[1], CV_32FC2);
    if (m_featureMode == KcfFeatureMode::Mkcf)
    {
        trainMKCF(image, 1.0F);
        trainStandardReference(m_tmpl, 1.0F);
    }
    else
    {
        train(m_tmpl, 1.0F);
    }
    initKF();
}

void CKcfTracker::initKF()
{
    m_filter = std::make_unique<CFilterKalman>(2);
    m_filter->m_A.eye();
    m_filter->m_B.eye();
    m_filter->m_H.eye();
    m_filter->m_R.eye();
    m_filter->m_Q.eye();
    m_Kal_init = false;
    m_Kalman_status_x = 0.0;
    m_Kalman_status_y = 0.0;
}

void CKcfTracker::setFusionOptions(const KcfFusionOptions &options)
{
    KcfFusionOptions normalized = options;
    switch (normalized.mode)
    {
    case KcfFusionMode::AutomaticKalman:
    case KcfFusionMode::ManualWeight:
    case KcfFusionMode::KcfOnly:
        break;
    default:
        normalized.mode = KcfFusionMode::AutomaticKalman;
        break;
    }
    normalized.simulationWeight = std::isfinite(normalized.simulationWeight)
        ? std::clamp(normalized.simulationWeight, 0.0, 1.0) : 0.5;
    if (normalized.mode == m_fusionOptions.mode
        && std::abs(normalized.simulationWeight - m_fusionOptions.simulationWeight) < 1e-12)
    {
        return;
    }

    m_fusionOptions = normalized;
    initKF();
    m_accum_x.clear();
    m_accum_y.clear();
    m_peakValueMaximum = 1.0F;
    m_peakValueMinimum = 0.85F;
}

void CKcfTracker::setSimulatedTemplate(const cv::Mat &image, const cv::Rect &roi)
{
    if (image.empty() || roi.width <= 0 || roi.height <= 0)
    {
        clearSimulatedTemplate();
        return;
    }

    image.copyTo(m_imageSim);
    normalizeImage(m_imageSim);
    m_imageSimROI.x = roi.x;
    m_imageSimROI.y = roi.y;
    m_imageSimROI.width = roi.width;
    m_imageSimROI.height = roi.height;
    const int imageWidth = m_imageSim.cols;
    const int imageHeight = m_imageSim.rows;
    m_imageSimROI.x = std::max(-m_imageSimROI.width + 1, std::min(m_imageSimROI.x, imageWidth - 1));
    m_imageSimROI.y = std::max(-m_imageSimROI.height + 1, std::min(m_imageSimROI.y, imageHeight - 1));
}

void CKcfTracker::clearSimulatedTemplate()
{
    m_imageSim.release();
    m_imageSimROI.x = 0;
    m_imageSimROI.y = 0;
    m_imageSimROI.width = 0;
    m_imageSimROI.height = 0;
}

bool CKcfTracker::hasSimulatedTemplate() const
{
    return !m_imageSim.empty() && m_imageSimROI.width > 0 && m_imageSimROI.height > 0;
}

const CKcfUpdateDiagnostic &CKcfTracker::lastUpdateDiagnostic() const
{
    return m_updateDiagnostic;
}

cv::Rect CKcfTracker::update(cv::Mat image)
{
    m_updateDiagnostic = {};
    m_updateDiagnostic.fusionMode = m_fusionOptions.mode;
    m_updateDiagnostic.simulationWeight = m_fusionOptions.simulationWeight;
    normalizeImage(image);
    clampRoiToImage(image);

    const cv::Point2f previousTopLeft(m_roi.x, m_roi.y);
    float peakValue = 0.0F;

    cv::Mat standardSearchFeature;
    cv::Point2f standardOffset(0.0F, 0.0F);
    float standardPeakValue = 0.0F;
    if (m_featureMode == KcfFeatureMode::Mkcf)
    {
        standardSearchFeature = getFeatures(image, false, 1.0F);
        standardOffset = detectWithModelAlpha(
            m_standardReferenceTmpl,
            m_standardReferenceAlphaF,
            standardSearchFeature,
            standardPeakValue);
    }

    cv::Point2f offset = (m_featureMode == KcfFeatureMode::Mkcf)
        ? detectMKCF(image, peakValue)
        : detect(m_tmpl, getFeatures(image, false, 1.0F), peakValue);

    if (m_featureMode == KcfFeatureMode::Mkcf
        && !m_standardReferenceTmpl.empty()
        && !m_standardReferenceAlphaF.empty())
    {
        constexpr float kMaxMkcfDeviationFromStandardPixel = 0.0F;
        const cv::Point2f delta = offset - standardOffset;
        const float deltaPixel = std::sqrt(delta.x * delta.x + delta.y * delta.y)
            * static_cast<float>(m_cell_size) * m_scale;
        if (deltaPixel > kMaxMkcfDeviationFromStandardPixel)
        {
            const float maxDeltaCell = kMaxMkcfDeviationFromStandardPixel
                / std::max(1e-6F, static_cast<float>(m_cell_size) * m_scale);
            const float deltaCell = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            offset = standardOffset + delta * (maxDeltaCell / std::max(1e-6F, deltaCell));
            peakValue = standardPeakValue;
        }
    }

    if (m_scale_step != 1.0F && m_featureMode != KcfFeatureMode::Mkcf)
    {
        float smallPeak = 0.0F;
        const cv::Point2f smallOffset =
            detect(m_tmpl, getFeatures(image, false, 1.0F / m_scale_step), smallPeak);
        if (m_scale_weight * smallPeak > peakValue)
        {
            offset = smallOffset;
            peakValue = smallPeak;
            m_scale /= m_scale_step;
            m_roi.width /= m_scale_step;
            m_roi.height /= m_scale_step;
        }

        float largePeak = 0.0F;
        const cv::Point2f largeOffset =
            detect(m_tmpl, getFeatures(image, false, m_scale_step), largePeak);
        if (m_scale_weight * largePeak > peakValue)
        {
            offset = largeOffset;
            peakValue = largePeak;
            m_scale *= m_scale_step;
            m_roi.width *= m_scale_step;
            m_roi.height *= m_scale_step;
        }
    }

    m_roi.x += offset.x * static_cast<float>(m_cell_size) * m_scale;
    m_roi.y += offset.y * static_cast<float>(m_cell_size) * m_scale;

    m_updateDiagnostic.hasResponse = true;
    m_updateDiagnostic.responsePeak = peakValue;
    m_updateDiagnostic.responseRoi = m_roi;
    if (std::abs(m_roi.x - previousTopLeft.x) > 0.5F * m_roi.width
        || std::abs(m_roi.y - previousTopLeft.y) > 0.5F * m_roi.height)
    {
        m_updateDiagnostic.correlationJumpRejected = true;
        m_roi.x = previousTopLeft.x;
        m_roi.y = previousTopLeft.y;
    }

    clampRoiToImage(image);

    if (!m_filter)
    {
        initKF();
    }

    m_updateDiagnostic.beforeFusionRoi = m_roi;
    if (m_fusionOptions.mode == KcfFusionMode::AutomaticKalman)
    {
        double tempX[2] = {0.0, 0.0}; //状态矩阵
        double tempY[2] = {0.0, 0.0}; //观测矩阵
        if (!m_Kal_init)
        {
            m_updateDiagnostic.kalmanSeeded = true;
            m_Kal_init = true;
            tempX[0] = tempY[0] = m_roi.x;
            tempX[1] = tempY[1] = m_roi.y;
            m_filter->m_X = RMatrix::Matrix(2, 1, tempX);
            m_filter->m_Y = RMatrix::Matrix(2, 1, tempY);
        }
        else
        {
            double tempU[2] = {0.0, 0.0};
            m_filter->m_u = RMatrix::Matrix(2, 1, tempU);

            if (peakValue > m_peakValueMaximum)
            {
                m_peakValueMaximum = peakValue;
            }
            if (peakValue < m_peakValueMinimum)
            {
                m_peakValueMinimum = peakValue;
            }
            const double peakRange = std::max(
                1e-12,
                static_cast<double>(m_peakValueMaximum - m_peakValueMinimum));
            const double normalizedPeak =
                (static_cast<double>(peakValue) - m_peakValueMinimum) / peakRange;
            if (normalizedPeak > 0.2)
            {
                m_filter->m_cR = m_filter->m_cRMaximum
                    - normalizedPeak * (m_filter->m_cRMaximum - m_filter->m_cRMinimum);
            }
            else
            {
                m_filter->m_cR = 1e-2 - normalizedPeak * (1e-2 - m_filter->m_cRMinimum);
            }

            tempY[0] = m_roi.x;
            tempY[1] = m_roi.y;
            if (m_Kalman_status_x != 0.0 && m_Kalman_status_y != 0.0)
            {
                tempX[0] = m_Kalman_status_x;
                tempX[1] = m_Kalman_status_y;
                m_filter->m_X = RMatrix::Matrix(2, 1, tempX);
                m_filter->m_Y = RMatrix::Matrix(2, 1, tempY);
                m_filter->Kalfilter();
                m_updateDiagnostic.fusionApplied = true;
                m_updateDiagnostic.normalizedPeak = normalizedPeak;
                m_updateDiagnostic.observationNoise = m_filter->m_cR;
                m_updateDiagnostic.gainX = m_filter->m_K.val[0][0];
                m_updateDiagnostic.gainY = m_filter->m_K.val[1][1];
            }
        }

        if (m_Kalman_status_x != 0.0 && m_Kalman_status_y != 0.0)
        {
            m_roi.x = static_cast<float>(m_filter->m_X.val[0][0]);
            m_roi.y = static_cast<float>(m_filter->m_X.val[1][0]);
        }
    }
    else if (m_fusionOptions.mode == KcfFusionMode::ManualWeight
        && std::isfinite(m_Kalman_status_x) && std::isfinite(m_Kalman_status_y))
    {
        // 手动比例直接作用于当前仿真先验与原始 KCF，不经过旧协方差重加权。
        const double simulationWeight = m_fusionOptions.simulationWeight;
        const double kcfWeight = 1.0 - simulationWeight;
        m_roi.x = static_cast<float>(simulationWeight * m_Kalman_status_x
            + kcfWeight * m_updateDiagnostic.beforeFusionRoi.x);
        m_roi.y = static_cast<float>(simulationWeight * m_Kalman_status_y
            + kcfWeight * m_updateDiagnostic.beforeFusionRoi.y);
        m_updateDiagnostic.fusionApplied = true;
        m_updateDiagnostic.gainX = kcfWeight;
        m_updateDiagnostic.gainY = kcfWeight;
    }

    if ((m_fusionOptions.mode == KcfFusionMode::AutomaticKalman
            && m_Kalman_status_x != 0.0 && m_Kalman_status_y != 0.0)
        || (m_fusionOptions.mode == KcfFusionMode::ManualWeight
            && m_updateDiagnostic.fusionApplied))
    {
        if (std::abs(m_roi.x - previousTopLeft.x) > 0.5F * m_roi.width
            || std::abs(m_roi.y - previousTopLeft.y) > 0.5F * m_roi.height)
        {
            m_updateDiagnostic.fusionJumpRejected = true;
            m_roi.x = previousTopLeft.x;
            m_roi.y = previousTopLeft.y;
        }
        clampRoiToImage(image);
    }

    cv::Mat currentFeature = (m_featureMode == KcfFeatureMode::FusedHogGray)
        ? fusedFeat(image, 1.0F)
        : getFeatures(image, false, 1.0F);
    if (m_featureMode == KcfFeatureMode::Mkcf)
    {
        if (m_numTmpl == 2)
        {
            train2TMKCF(image, m_interp_factor, 0.0, 1.0);
        }
        else
        {
            train3TMKCF(image, m_interp_factor, 0.0, 1.0);
        }
        trainStandardReference(currentFeature, m_interp_factor);
    }
    else
    {
        train(currentFeature, m_interp_factor);
    }

    m_updateDiagnostic.finalRoi = m_roi;
    m_updateDiagnostic.completed = true;
    ++m_imageCount;
    return toIntegerRect(m_roi);
}

cv::Point2f CKcfTracker::detect(const cv::Mat &model, const cv::Mat &features, float &peakValue)
{
    if (model.empty() || features.empty() || model.size() != features.size())
    {
        peakValue = 0.0F;
        return cv::Point2f(0.0F, 0.0F);
    }

    cv::Mat kernel = gaussianCorrelation(features, model);
    cv::Mat response =
        KcfFftTools::real(KcfFftTools::fftd(
            KcfFftTools::complexMultiplication(m_alphaF, KcfFftTools::fftd(kernel)),
            true));

    cv::Point maxLocation;
    double maxValue = 0.0;
    cv::minMaxLoc(response, nullptr, &maxValue, nullptr, &maxLocation);
    peakValue = static_cast<float>(maxValue);

    cv::Point2f peak(static_cast<float>(maxLocation.x), static_cast<float>(maxLocation.y));
    if (maxLocation.x > 0 && maxLocation.x < response.cols - 1)
    {
        peak.x += subPixelPeak(
            response.at<float>(maxLocation.y, maxLocation.x - 1),
            peakValue,
            response.at<float>(maxLocation.y, maxLocation.x + 1));
    }
    if (maxLocation.y > 0 && maxLocation.y < response.rows - 1)
    {
        peak.y += subPixelPeak(
            response.at<float>(maxLocation.y - 1, maxLocation.x),
            peakValue,
            response.at<float>(maxLocation.y + 1, maxLocation.x));
    }

    peak.x -= static_cast<float>(response.cols) * 0.5F;
    peak.y -= static_cast<float>(response.rows) * 0.5F;
    return peak;
}

cv::Point2f CKcfTracker::detectWithModelAlpha(
    const cv::Mat &model,
    const cv::Mat &alphaF,
    const cv::Mat &features,
    float &peakValue)
{
    if (model.empty() || alphaF.empty() || features.empty() || model.size() != features.size())
    {
        peakValue = 0.0F;
        return cv::Point2f(0.0F, 0.0F);
    }

    cv::Mat kernel = gaussianCorrelation(features, model);
    cv::Mat response =
        KcfFftTools::real(KcfFftTools::fftd(
            KcfFftTools::complexMultiplication(alphaF, KcfFftTools::fftd(kernel)),
            true));

    cv::Point maxLocation;
    double maxValue = 0.0;
    cv::minMaxLoc(response, nullptr, &maxValue, nullptr, &maxLocation);
    peakValue = static_cast<float>(maxValue);

    cv::Point2f peak(static_cast<float>(maxLocation.x), static_cast<float>(maxLocation.y));
    if (maxLocation.x > 0 && maxLocation.x < response.cols - 1)
    {
        peak.x += subPixelPeak(
            response.at<float>(maxLocation.y, maxLocation.x - 1),
            peakValue,
            response.at<float>(maxLocation.y, maxLocation.x + 1));
    }
    if (maxLocation.y > 0 && maxLocation.y < response.rows - 1)
    {
        peak.y += subPixelPeak(
            response.at<float>(maxLocation.y - 1, maxLocation.x),
            peakValue,
            response.at<float>(maxLocation.y + 1, maxLocation.x));
    }

    peak.x -= static_cast<float>(response.cols) * 0.5F;
    peak.y -= static_cast<float>(response.rows) * 0.5F;
    return peak;
}

void CKcfTracker::train(const cv::Mat &features, float trainInterpFactor)
{
    if (features.empty())
    {
        return;
    }

    const cv::Mat kernel = gaussianCorrelation(features, features);
    const cv::Mat alpha =
        KcfFftTools::complexDivision(m_prob, KcfFftTools::fftd(kernel) + m_lambda);
    if (m_tmpl.empty() || m_tmpl.size() != features.size())
    {
        m_tmpl = features.clone();
    }
    else
    {
        m_tmpl = (1.0F - trainInterpFactor) * m_tmpl + trainInterpFactor * features;
    }

    if (m_alphaF.empty() || m_alphaF.size() != alpha.size())
    {
        m_alphaF = alpha.clone();
    }
    else
    {
        m_alphaF = (1.0F - trainInterpFactor) * m_alphaF + trainInterpFactor * alpha;
    }
}

void CKcfTracker::trainStandardReference(const cv::Mat &features, float trainInterpFactor)
{
    if (features.empty())
    {
        return;
    }

    const cv::Mat kernel = gaussianCorrelation(features, features);
    const cv::Mat alpha =
        KcfFftTools::complexDivision(m_prob, KcfFftTools::fftd(kernel) + m_lambda);
    if (m_standardReferenceTmpl.empty() || m_standardReferenceTmpl.size() != features.size())
    {
        m_standardReferenceTmpl = features.clone();
    }
    else
    {
        m_standardReferenceTmpl =
            (1.0F - trainInterpFactor) * m_standardReferenceTmpl + trainInterpFactor * features;
    }

    if (m_standardReferenceAlphaF.empty() || m_standardReferenceAlphaF.size() != alpha.size())
    {
        m_standardReferenceAlphaF = alpha.clone();
    }
    else
    {
        m_standardReferenceAlphaF =
            (1.0F - trainInterpFactor) * m_standardReferenceAlphaF + trainInterpFactor * alpha;
    }
}

cv::Mat CKcfTracker::gaussianCorrelation(const cv::Mat &left, const cv::Mat &right) const
{
    cv::Mat leftFloat;
    cv::Mat rightFloat;
    left.convertTo(leftFloat, CV_32F);
    right.convertTo(rightFloat, CV_32F);

    cv::Mat correlation = cv::Mat::zeros(m_sizePatch[0], m_sizePatch[1], CV_32F);
    const int channels = std::max(1, m_sizePatch[2]);
    for (int channel = 0; channel < channels; ++channel)
    {
        cv::Mat leftChannel = leftFloat.row(channel).reshape(1, m_sizePatch[0]);
        cv::Mat rightChannel = rightFloat.row(channel).reshape(1, m_sizePatch[0]);
        cv::Mat spectrum;
        cv::mulSpectrums(
            KcfFftTools::fftd(leftChannel),
            KcfFftTools::fftd(rightChannel),
            spectrum,
            0,
            true);
        cv::Mat spatial = KcfFftTools::fftd(spectrum, true);
        KcfFftTools::rearrange(spatial);
        correlation += KcfFftTools::real(spatial);
    }

    const double leftNorm = cv::sum(leftFloat.mul(leftFloat))[0];
    const double rightNorm = cv::sum(rightFloat.mul(rightFloat))[0];
    cv::Mat distance =
        static_cast<float>(leftNorm + rightNorm) - 2.0F * correlation;
    distance /= static_cast<float>(m_sizePatch[0] * m_sizePatch[1] * channels);
    cv::max(distance, 0.0F, distance);
    distance /= m_sigma * m_sigma;
    cv::exp(-distance, distance);
    return distance;
}

cv::Mat CKcfTracker::gaussianCorrelation(
    const cv::Mat &left,
    const cv::Mat &right,
    int height,
    int width,
    float sigma) const
{
    if (left.empty() || right.empty() || left.cols != right.cols || left.cols != height * width)
    {
        return cv::Mat::zeros(height, width, CV_32F);
    }

    cv::Mat leftFloat;
    cv::Mat rightFloat;
    left.convertTo(leftFloat, CV_32F);
    right.convertTo(rightFloat, CV_32F);

    cv::Mat spectrumSum = cv::Mat::zeros(height, width, CV_32FC2);
    const int channelCount = leftFloat.rows;
    for (int channel = 0; channel < channelCount; ++channel)
    {
        cv::Mat leftChannel = leftFloat.row(channel).reshape(1, height);
        cv::Mat rightChannel = rightFloat.row(channel).reshape(1, height);
        cv::Mat spectrum;
        cv::mulSpectrums(
            KcfFftTools::fftd(leftChannel),
            KcfFftTools::fftd(rightChannel),
            spectrum,
            0,
            true);
        spectrumSum += spectrum;
    }

    cv::Mat correlation = KcfFftTools::fftd(spectrumSum, true);
    KcfFftTools::rearrange(correlation);
    cv::Mat realCorrelation = KcfFftTools::real(correlation);

    const double leftNorm = cv::sum(leftFloat.mul(leftFloat))[0];
    const double rightNorm = cv::sum(rightFloat.mul(rightFloat))[0];
    cv::Mat distance = static_cast<float>(leftNorm + rightNorm) - 2.0F * realCorrelation;
    distance /= static_cast<float>(height * width * std::max(1, channelCount));
    cv::max(distance, 0.0F, distance);
    distance /= sigma * sigma;
    cv::exp(-distance, distance);
    return distance;
}

cv::Mat CKcfTracker::createGaussianPeak(int sizeY, int sizeX) const
{
    cv::Mat output(sizeY, sizeX, CV_32F);
    const float outputSigma =
        std::sqrt(static_cast<float>(sizeX * sizeY))
        / static_cast<float>(m_padding) * m_output_sigma_factor;
    const float mult = -0.5F / (outputSigma * outputSigma);

    for (int y = 0; y < sizeY; ++y)
    {
        for (int x = 0; x < sizeX; ++x)
        {
            const int shiftedY = y - sizeY / 2;
            const int shiftedX = x - sizeX / 2;
            output.at<float>(y, x) =
                std::exp(mult * static_cast<float>(shiftedY * shiftedY + shiftedX * shiftedX));
        }
    }

    return KcfFftTools::fftd(output);
}

cv::Mat CKcfTracker::getFeatures(const cv::Mat &image, bool initHann, float scaleAdjust)
{
    if (initHann || m_tmplSize.width <= 0 || m_tmplSize.height <= 0)
    {
        initializeTemplateGeometryFromRoi(m_roi);
    }

    if (m_hogFeatures)
    {
        cv::Mat patch = extractPatchImage(image, m_roi, scaleAdjust);
        if (patch.depth() != CV_8U)
        {
            patch.convertTo(patch, CV_8U);
        }

        IplImage patchIpl;
        cvInitImageHeader(
            &patchIpl,
            cvSize(patch.cols, patch.rows),
            IPL_DEPTH_8U,
            patch.channels());
        cvSetData(&patchIpl, patch.data, static_cast<int>(patch.step));
        CvLSVMFeatureMapCaskade *map = nullptr;
        getFeatureMaps(&patchIpl, m_cell_size, &map);
        normalizeAndTruncate(map, VAL_OF_TRUNCATE);
        PCAFeatureMaps(map);

        m_sizePatch[0] = map->sizeY;
        m_sizePatch[1] = map->sizeX;
        m_sizePatch[2] = map->numFeatures;

        cv::Mat featureMap(
            cv::Size(map->numFeatures, map->sizeX * map->sizeY),
            CV_32F,
            map->map);
        cv::Mat transposedFeatureMap = featureMap.t();
        cv::Mat feature = transposedFeatureMap.clone();
        freeFeatureMapObject(&map);

        if (m_labFeatures && patch.channels() == 3)
        {
            cv::Mat imgLab;
            cv::cvtColor(patch, imgLab, cv::COLOR_BGR2Lab);
            const unsigned char *input = imgLab.data;
            cv::Mat outputLab(
                KcfLabData::kClusterCount,
                m_sizePatch[0] * m_sizePatch[1],
                CV_32F,
                cv::Scalar(0.0F));

            int cntCell = 0;
            for (int cellY = m_cell_size; cellY < patch.rows - m_cell_size; cellY += m_cell_size)
            {
                for (int cellX = m_cell_size; cellX < patch.cols - m_cell_size; cellX += m_cell_size)
                {
                    for (int y = cellY; y < cellY + m_cell_size; ++y)
                    {
                        for (int x = cellX; x < cellX + m_cell_size; ++x)
                        {
                            const float l = static_cast<float>(input[(patch.cols * y + x) * 3]);
                            const float a = static_cast<float>(input[(patch.cols * y + x) * 3 + 1]);
                            const float b = static_cast<float>(input[(patch.cols * y + x) * 3 + 2]);
                            float minDist = std::numeric_limits<float>::max();
                            int minIdx = 0;
                            for (int idx = 0; idx < KcfLabData::kClusterCount; ++idx)
                            {
                                const float *centroid = KcfLabData::kCenters[idx];
                                const float dist =
                                    (l - centroid[0]) * (l - centroid[0])
                                    + (a - centroid[1]) * (a - centroid[1])
                                    + (b - centroid[2]) * (b - centroid[2]);
                                if (dist < minDist)
                                {
                                    minDist = dist;
                                    minIdx = idx;
                                }
                            }
                            if (cntCell < outputLab.cols)
                            {
                                outputLab.at<float>(minIdx, cntCell) += 1.0F / static_cast<float>(m_cell_sizeQ);
                            }
                        }
                    }
                    ++cntCell;
                }
            }

            m_sizePatch[2] += outputLab.rows;
            feature.push_back(outputLab);
        }

        if (initHann || m_hann.empty())
        {
            createHanningMats();
        }
        if (!m_hann.empty() && m_hann.size() == feature.size())
        {
            feature = feature.mul(m_hann);
        }
        return feature;
    }

    cv::Mat feature = extractPatchFeature(image, m_roi, scaleAdjust);
    m_sizePatch[0] = feature.rows;
    m_sizePatch[1] = feature.cols;
    m_sizePatch[2] = 1;
    feature = feature.reshape(1, 1);
    if (initHann || m_hann.empty())
    {
        createHanningMats();
    }
    if (!m_hann.empty() && m_hann.size() == feature.size())
    {
        feature = feature.mul(m_hann);
    }
    return feature;
}

void CKcfTracker::getFeatures(
    const cv::Mat &image,
    const cv::Rect &roi,
    float scaleAdjust,
    cv::Mat &primaryFeature,
    cv::Mat &secondaryFeature)
{
    cv::Rect_<float> savedRoi = m_roi;
    m_roi = roi;
    if (m_tmplSize.width <= 0 || m_tmplSize.height <= 0)
    {
        initializeTemplateGeometryFromRoi(m_roi);
    }
    cv::Mat patch = extractPatchImage(image, m_roi, scaleAdjust);
    m_roi = savedRoi;
    if (patch.depth() != CV_8U)
    {
        patch.convertTo(patch, CV_8U);
    }

    IplImage patchIpl;
    cvInitImageHeader(
        &patchIpl,
        cvSize(patch.cols, patch.rows),
        IPL_DEPTH_8U,
        patch.channels());
    cvSetData(&patchIpl, patch.data, static_cast<int>(patch.step));
    CvLSVMFeatureMapCaskade *featureMap = nullptr;
    getFeatureMaps(&patchIpl, m_cell_size, &featureMap);
    normalizeAndTruncate(featureMap, VAL_OF_TRUNCATE);
    PCAFeatureMaps(featureMap);

    const int height = featureMap->sizeY;
    const int width = featureMap->sizeX;
    const int channels = featureMap->numFeatures;
    m_sizePatch[0] = height;
    m_sizePatch[1] = width;
    m_sizePatch[2] = channels;
    cv::Mat rawFeature(
        cv::Size(channels, width * height),
        CV_32F,
        featureMap->map);
    cv::transpose(rawFeature, primaryFeature);
    freeFeatureMapObject(&featureMap);

    cv::Mat gray = KcfRectTools::getGrayImage(patch);
    gray.convertTo(gray, CV_32F, 1.0 / 255.0);
    gray -= 0.5F;
    cv::Mat grayResized;
    cv::resize(gray, grayResized, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
    secondaryFeature = grayResized.clone();

    createHannUnified(height, width);
    if (!m_hannCol.empty() && m_hannCol.cols == primaryFeature.cols)
    {
        for (int row = 0; row < primaryFeature.rows; ++row)
        {
            primaryFeature.row(row) = primaryFeature.row(row).mul(m_hannCol);
        }
    }
    if (!m_hann2d.empty() && m_hann2d.size() == secondaryFeature.size())
    {
        secondaryFeature = secondaryFeature.mul(m_hann2d);
    }
    secondaryFeature = secondaryFeature.reshape(1, 1);
}

cv::Mat CKcfTracker::fusedFeat(const cv::Mat &image, float scaleAdjust)
{
    cv::Mat hogFeature;
    cv::Mat grayFeature;
    getFeatures(image, toIntegerRect(m_roi), scaleAdjust, hogFeature, grayFeature);
    cv::Mat output = concatFeature(hogFeature, grayFeature);
    m_sizePatch[0] = 18;
    m_sizePatch[1] = 24;
    m_sizePatch[2] = output.rows;
    return output;
}

cv::Mat CKcfTracker::concatFeature(const cv::Mat &hogFeature, const cv::Mat &grayFeature) const
{
    constexpr int kLegacyHogChannels = 31;
    const cv::Size legacyGrid(24, 18);
    const int featureDim = legacyGrid.area();
    cv::Mat output = cv::Mat::zeros(kLegacyHogChannels + 1, featureDim, CV_32F);
    const auto chooseFeatureRows = [this](int elementCount) {
        if (m_sizePatch[0] > 0 && elementCount % m_sizePatch[0] == 0)
        {
            return m_sizePatch[0];
        }
        if (m_sizePatch[1] > 0 && elementCount % m_sizePatch[1] == 0)
        {
            return m_sizePatch[1];
        }
        int rows = static_cast<int>(std::sqrt(static_cast<double>(std::max(1, elementCount))));
        while (rows > 1 && elementCount % rows != 0)
        {
            --rows;
        }
        return std::max(1, rows);
    };

    const int hogRows = std::min(kLegacyHogChannels, hogFeature.rows);
    for (int row = 0; row < hogRows; ++row)
    {
        cv::Mat channel = hogFeature.row(row).reshape(1, chooseFeatureRows(hogFeature.cols));
        channel.convertTo(channel, CV_32F);
        cv::Mat resized;
        cv::resize(channel, resized, legacyGrid, 0.0, 0.0, cv::INTER_AREA);
        double minValue = 0.0;
        double maxValue = 0.0;
        cv::minMaxLoc(resized, &minValue, &maxValue);
        resized = (resized - static_cast<float>(minValue))
            / static_cast<float>(maxValue - minValue + 1e-6);
        resized.reshape(1, 1).copyTo(output.row(row));
    }

    cv::Mat gray = grayFeature.reshape(1, 1);
    cv::Mat grayImage = gray;
    if (m_sizePatch[0] > 0 && gray.cols == m_sizePatch[0] * m_sizePatch[1])
    {
        grayImage = gray.reshape(1, m_sizePatch[0]);
    }
    cv::Mat grayResized;
    cv::resize(grayImage, grayResized, legacyGrid, 0.0, 0.0, cv::INTER_AREA);
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(grayResized, &minValue, &maxValue);
    grayResized = (grayResized - static_cast<float>(minValue))
        / static_cast<float>(maxValue - minValue + 1e-6);
    grayResized.reshape(1, 1).copyTo(output.row(kLegacyHogChannels));
    return output;
}

void CKcfTracker::createHanningMats()
{
    cv::Mat hannX(1, m_sizePatch[1], CV_32F);
    cv::Mat hannY(m_sizePatch[0], 1, CV_32F);
    const double pi = 3.14159265358979323846;

    for (int x = 0; x < hannX.cols; ++x)
    {
        hannX.at<float>(0, x) =
            0.5F * (1.0F - static_cast<float>(std::cos(2.0 * pi * x / (hannX.cols - 1))));
    }
    for (int y = 0; y < hannY.rows; ++y)
    {
        hannY.at<float>(y, 0) =
            0.5F * (1.0F - static_cast<float>(std::cos(2.0 * pi * y / (hannY.rows - 1))));
    }

    const cv::Mat hann2d = hannY * hannX;
    m_hann = cv::Mat(m_sizePatch[2], m_sizePatch[0] * m_sizePatch[1], CV_32F);
    const cv::Mat flat = hann2d.reshape(1, 1);
    for (int channel = 0; channel < m_sizePatch[2]; ++channel)
    {
        flat.copyTo(m_hann.row(channel));
    }
}

void CKcfTracker::createHannUnified(int height, int width)
{
    if (!m_hann2d.empty() && m_hann2d.rows == height && m_hann2d.cols == width)
    {
        return;
    }

    cv::Mat hannX(1, width, CV_32F);
    cv::Mat hannY(height, 1, CV_32F);
    const double pi = 3.14159265358979323846;
    for (int x = 0; x < width; ++x)
    {
        hannX.at<float>(0, x) =
            0.5F * (1.0F - static_cast<float>(std::cos(2.0 * pi * x / std::max(1, width - 1))));
    }
    for (int y = 0; y < height; ++y)
    {
        hannY.at<float>(y, 0) =
            0.5F * (1.0F - static_cast<float>(std::cos(2.0 * pi * y / std::max(1, height - 1))));
    }
    m_hann2d = hannY * hannX;
    m_hannCol = m_hann2d.reshape(1, 1).clone();
}

float CKcfTracker::subPixelPeak(float left, float center, float right) const
{
    const float divisor = 2.0F * center - right - left;
    if (std::abs(divisor) < std::numeric_limits<float>::epsilon())
    {
        return 0.0F;
    }
    return 0.5F * (right - left) / divisor;
}

cv::Point2f CKcfTracker::detectMKCF(const cv::Mat &image, float &peakResponse)
{
    if (m_histTmplPrimary.empty() || m_histTmplSecondary.empty() || m_alphaF.empty())
    {
        cv::Mat feature = getFeatures(image, false, 1.0F);
        return detect(m_tmpl, feature, peakResponse);
    }

    const int featureHeight = m_alphaF.rows;
    const int featureWidth = m_alphaF.cols;
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    getFeatures(image, toIntegerRect(m_roi), 1.0F, currentPrimary, currentSecondary);

    cv::Mat corrPrimary = gaussianCorrelation(
        currentPrimary,
        m_histTmplPrimary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary);
    cv::Mat corrSecondary = gaussianCorrelation(
        currentSecondary,
        m_histTmplSecondary,
        featureHeight,
        featureWidth,
        m_sigmaSecondary);
    cv::Mat corrCombined =
        m_kernelCoeffPrimary * corrPrimary + m_kernelCoeffSecondary * corrSecondary;
    cv::Mat response = KcfFftTools::real(KcfFftTools::fftd(
        KcfFftTools::complexMultiplication(m_alphaF, KcfFftTools::fftd(corrCombined)),
        true));

    cv::Point peakInt;
    double peakValue = 0.0;
    cv::minMaxLoc(response, nullptr, &peakValue, nullptr, &peakInt);
    peakResponse = static_cast<float>(peakValue);

    cv::Point2f peak(static_cast<float>(peakInt.x), static_cast<float>(peakInt.y));
    if (peakInt.x > 0 && peakInt.x < response.cols - 1)
    {
        peak.x += subPixelPeak(
            response.at<float>(peakInt.y, peakInt.x - 1),
            peakResponse,
            response.at<float>(peakInt.y, peakInt.x + 1));
    }
    if (peakInt.y > 0 && peakInt.y < response.rows - 1)
    {
        peak.y += subPixelPeak(
            response.at<float>(peakInt.y - 1, peakInt.x),
            peakResponse,
            response.at<float>(peakInt.y + 1, peakInt.x));
    }
    peak.x -= static_cast<float>(response.cols) * 0.5F;
    peak.y -= static_cast<float>(response.rows) * 0.5F;
    return peak;
}

void CKcfTracker::trainMKCF(const cv::Mat &image, float trainInterpFactor)
{
    const int featureHeight = m_alphaF.rows;
    const int featureWidth = m_alphaF.cols;

    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    getFeatures(image, toIntegerRect(m_roi), 1.0F, currentPrimary, currentSecondary);

    if (m_histTmplPrimary.empty() || m_histTmplPrimary.size() != currentPrimary.size())
    {
        m_histTmplPrimary = currentPrimary.clone();
    }
    if (m_histTmplSecondary.empty() || m_histTmplSecondary.size() != currentSecondary.size())
    {
        m_histTmplSecondary = currentSecondary.clone();
    }

    cv::Mat kernelPrimary = KcfFftTools::fftd(gaussianCorrelation(
        currentPrimary,
        currentPrimary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary));
    cv::Mat kernelSecondary = KcfFftTools::fftd(gaussianCorrelation(
        currentSecondary,
        currentSecondary,
        featureHeight,
        featureWidth,
        m_sigmaSecondary));

    m_kernelCoeffPrimary = 0.9F;
    m_kernelCoeffSecondary = 0.1F;

    float previousKernelCoeffPrimary = m_kernelCoeffPrimary;
    float previousKernelCoeffSecondary = m_kernelCoeffSecondary;
    cv::Mat previousAlphaF = m_alphaF.clone();
    constexpr float kConvergeThreshold = 0.03F;
    constexpr int kMaxIterations = 1;

    for (int iteration = 0; iteration < kMaxIterations; ++iteration)
    {
        cv::Mat kernelLinear =
            kernelPrimary * m_kernelCoeffPrimary + kernelSecondary * m_kernelCoeffSecondary;

        cv::Mat denominator = addRealScalar(kernelLinear, m_lambda);
        denominator = addRealScalar(denominator, 1e-7F);

        cv::Mat currentAlphaF = KcfFftTools::complexDivision(m_prob, denominator);
        m_alphaF = (1.0F - trainInterpFactor) * m_alphaF + trainInterpFactor * currentAlphaF;

        const cv::Mat &probability = m_prob;
        const cv::Mat &alphaFrequency = m_alphaF;

        const double bPrimary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(probability),
            KcfFftTools::complexMultiplication(kernelPrimary, alphaFrequency)));
        const double bSecondary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(probability),
            KcfFftTools::complexMultiplication(kernelSecondary, alphaFrequency)));

        const double vPrimary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(alphaFrequency),
            KcfFftTools::complexMultiplication(kernelPrimary, alphaFrequency)));
        const double vSecondary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(alphaFrequency),
            KcfFftTools::complexMultiplication(kernelSecondary, alphaFrequency)));

        const cv::Mat kernelPrimarySquare =
            KcfFftTools::complexMultiplication(kernelPrimary, kernelPrimary);
        const cv::Mat kernelCross =
            KcfFftTools::complexMultiplication(kernelPrimary, kernelSecondary);
        const cv::Mat kernelSecondarySquare =
            KcfFftTools::complexMultiplication(kernelSecondary, kernelSecondary);

        const double gPrimaryPrimary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(alphaFrequency),
            KcfFftTools::complexMultiplication(kernelPrimarySquare, alphaFrequency)));
        const double gPrimarySecondary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(alphaFrequency),
            KcfFftTools::complexMultiplication(kernelCross, alphaFrequency)));
        const double gSecondarySecondary = sumRealPart(KcfFftTools::complexMultiplication(
            KcfFftTools::complexConj(alphaFrequency),
            KcfFftTools::complexMultiplication(kernelSecondarySquare, alphaFrequency)));

        const double penaltyCoeff = m_upsilon;
        const double matB11 = gPrimaryPrimary + penaltyCoeff;
        const double matB12 = gPrimarySecondary + penaltyCoeff;
        const double matB22 = gSecondarySecondary + penaltyCoeff;

        const double vecC1 = bPrimary - 0.5 * m_lambda * vPrimary + penaltyCoeff;
        const double vecC2 = bSecondary - 0.5 * m_lambda * vSecondary + penaltyCoeff;

        cv::Vec2d coeffFull(0.0, 0.0);
        const double determinant = matB11 * matB22 - matB12 * matB12;
        const bool fullSolutionValid = std::abs(determinant) > 1e-12;
        if (fullSolutionValid)
        {
            coeffFull[0] = (matB22 * vecC1 - matB12 * vecC2) / determinant;
            coeffFull[1] = (-matB12 * vecC1 + matB11 * vecC2) / determinant;
        }

        auto calcObjectiveFunc = [&](const cv::Vec2d &coeff) -> double {
            const double quadraticTerm =
                0.5 * (coeff[0] * (gPrimaryPrimary * coeff[0] + gPrimarySecondary * coeff[1])
                    + coeff[1] * (gPrimarySecondary * coeff[0] + gSecondarySecondary * coeff[1]));
            const double linearTerm =
                -(bPrimary * coeff[0] + bSecondary * coeff[1])
                + 0.5 * m_lambda * (vPrimary * coeff[0] + vSecondary * coeff[1]);
            const double penaltyTerm =
                0.5 * penaltyCoeff * std::pow((coeff[0] + coeff[1] - 1.0), 2);
            return quadraticTerm + linearTerm + penaltyTerm;
        };

        cv::Vec2d bestCoeff(0.0, 0.0);
        double bestObjectiveValue = 1e300;
        bool hasValidSolution = false;

        if (fullSolutionValid && coeffFull[0] >= 0.0 && coeffFull[1] >= 0.0)
        {
            bestCoeff = coeffFull;
            bestObjectiveValue = calcObjectiveFunc(bestCoeff);
            hasValidSolution = true;
        }

        if (matB11 > 1e-12)
        {
            cv::Vec2d primaryOnly(vecC1 / matB11, 0.0);
            if (primaryOnly[0] >= 0.0)
            {
                const double objectiveValue = calcObjectiveFunc(primaryOnly);
                if (!hasValidSolution || objectiveValue < bestObjectiveValue)
                {
                    bestCoeff = primaryOnly;
                    bestObjectiveValue = objectiveValue;
                    hasValidSolution = true;
                }
            }
        }

        if (matB22 > 1e-12)
        {
            cv::Vec2d secondaryOnly(0.0, vecC2 / matB22);
            if (secondaryOnly[1] >= 0.0)
            {
                const double objectiveValue = calcObjectiveFunc(secondaryOnly);
                if (!hasValidSolution || objectiveValue < bestObjectiveValue)
                {
                    bestCoeff = secondaryOnly;
                    bestObjectiveValue = objectiveValue;
                    hasValidSolution = true;
                }
            }
        }

        if (!hasValidSolution)
        {
            bestCoeff[0] = std::max(0.0, coeffFull[0]);
            bestCoeff[1] = std::max(0.0, coeffFull[1]);
            const double coeffSum = bestCoeff[0] + bestCoeff[1];
            if (coeffSum <= 1e-12)
            {
                bestCoeff = cv::Vec2d(1.0, 0.0);
            }
            else
            {
                bestCoeff /= coeffSum;
            }
        }

        m_kernelCoeffPrimary = static_cast<float>(
            bestCoeff[0] * trainInterpFactor + (1.0F - trainInterpFactor) * m_kernelCoeffPrimary);
        m_kernelCoeffSecondary = static_cast<float>(
            bestCoeff[1] * trainInterpFactor + (1.0F - trainInterpFactor) * m_kernelCoeffSecondary);

        const float coeffTotal = m_kernelCoeffPrimary + m_kernelCoeffSecondary;
        if (coeffTotal > 1e-12F)
        {
            m_kernelCoeffPrimary /= coeffTotal;
            m_kernelCoeffSecondary /= coeffTotal;
        }

        if (iteration > 1)
        {
            const double alphaRelativeChange =
                cv::norm(m_alphaF - previousAlphaF) / (cv::norm(previousAlphaF) + 1e-9);
            const float coeffDeltaPrimary =
                std::abs(m_kernelCoeffPrimary - previousKernelCoeffPrimary);
            const float coeffDeltaSecondary =
                std::abs(m_kernelCoeffSecondary - previousKernelCoeffSecondary);
            if (alphaRelativeChange <= kConvergeThreshold
                && (coeffDeltaPrimary + coeffDeltaSecondary) <= kConvergeThreshold)
            {
                break;
            }
        }

        previousAlphaF = m_alphaF.clone();
        previousKernelCoeffPrimary = m_kernelCoeffPrimary;
        previousKernelCoeffSecondary = m_kernelCoeffSecondary;
    }

    m_histTmplPrimary =
        trainInterpFactor * currentPrimary + (1.0F - trainInterpFactor) * m_histTmplPrimary;
    m_histTmplSecondary =
        trainInterpFactor * currentSecondary + (1.0F - trainInterpFactor) * m_histTmplSecondary;
    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
}

void CKcfTracker::train2TMKCF(
    const cv::Mat &image,
    float trainInterpFactor,
    double mu,
    double muInc)
{
    if (m_imageSim.empty() || m_imageSimROI.width <= 0 || m_imageSimROI.height <= 0)
    {
        trainMKCF(image, trainInterpFactor);
        return;
    }

    const int featureHeight = std::max(1, m_alphaF.rows);
    const int featureWidth = std::max(1, m_alphaF.cols);
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    cv::Mat simPrimary;
    cv::Mat simSecondary;
    getFeatures(image, toIntegerRect(m_roi), 1.0F, currentPrimary, currentSecondary);
    getFeatures(m_imageSim, m_imageSimROI, 1.0F, simPrimary, simSecondary);
    if (currentPrimary.size() != simPrimary.size() || currentSecondary.size() != simSecondary.size())
    {
        trainMKCF(image, trainInterpFactor);
        return;
    }

    if (m_histTmplPrimary.empty() || m_histTmplPrimary.size() != currentPrimary.size())
    {
        m_histTmplPrimary = currentPrimary.clone();
    }
    if (m_histTmplSecondary.empty() || m_histTmplSecondary.size() != currentSecondary.size())
    {
        m_histTmplSecondary = currentSecondary.clone();
    }

    computeKernels(
        currentPrimary,
        currentSecondary,
        simPrimary,
        simSecondary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary,
        m_sigmaSecondary);

    cv::Mat alphaTemplate1 = solveSingleTemplateAlpha(
        m_kernelCoeffPrimary,
        m_kernelCoeffSecondary,
        m_kernel11Primary,
        m_kernel11Secondary);
    cv::Mat alphaTemplate2 = solveSingleTemplateAlpha(
        m_kernelCoeffPrimary,
        m_kernelCoeffSecondary,
        m_kernel22Primary,
        m_kernel22Secondary);

    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
    float previousPrimary = m_kernelCoeffPrimary;
    float previousSecondary = m_kernelCoeffSecondary;
    std::deque<double> alphaDiffHistory;
    constexpr int kMaxIterations = 1;
    for (int iteration = 0; iteration < kMaxIterations; ++iteration)
    {
        const cv::Mat kernel11 =
            m_kernel11Primary * m_kernelCoeffPrimary + m_kernel11Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel22 =
            m_kernel22Primary * m_kernelCoeffPrimary + m_kernel22Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel12 =
            m_kernel12Primary * m_kernelCoeffPrimary + m_kernel12Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel21 =
            m_kernel21Primary * m_kernelCoeffPrimary + m_kernel21Secondary * m_kernelCoeffSecondary;

        cv::Mat denom1 = addRealScalar(kernel11, m_lambda + static_cast<float>(mu));
        denom1 = addRealScalar(denom1, 1e-7F);
        cv::Mat denom2 = addRealScalar(kernel22, m_lambda + static_cast<float>(mu));
        denom2 = addRealScalar(denom2, 1e-7F);
        alphaTemplate1 = KcfFftTools::complexDivision(
            complexAddition(
                m_prob,
                complexMultiplicationScalar(
                    KcfFftTools::complexMultiplication(
                        KcfFftTools::complexDivision(kernel12, kernel11),
                        alphaTemplate2),
                    static_cast<float>(mu))),
            denom1);
        alphaTemplate2 = KcfFftTools::complexDivision(
            complexAddition(
                m_prob,
                complexMultiplicationScalar(
                    KcfFftTools::complexMultiplication(
                        KcfFftTools::complexDivision(kernel21, kernel22),
                        alphaTemplate1),
                    static_cast<float>(mu))),
            denom2);

        cv::Mat alpha1Abs2;
        cv::Mat alpha2Abs2;
        computeAbs2(alphaTemplate1, alpha1Abs2);
        computeAbs2(alphaTemplate2, alpha2Abs2);

        const double uPrimary =
            sumAbs2MulRealK(alpha1Abs2, m_kernel11Primary)
            + sumAbs2MulRealK(alpha2Abs2, m_kernel22Primary);
        const double uSecondary =
            sumAbs2MulRealK(alpha1Abs2, m_kernel11Secondary)
            + sumAbs2MulRealK(alpha2Abs2, m_kernel22Secondary);
        const double cPrimary = sumConjAMulKMulB(alphaTemplate1, m_kernel12Primary, alphaTemplate2);
        const double cSecondary = sumConjAMulKMulB(alphaTemplate1, m_kernel12Secondary, alphaTemplate2);
        const double bPrimary =
            sumConjAMulKMulProb(alphaTemplate1, m_kernel11Primary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate2, m_kernel22Primary, m_prob);
        const double bSecondary =
            sumConjAMulKMulProb(alphaTemplate1, m_kernel11Secondary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate2, m_kernel22Secondary, m_prob);

        double aPP = 0.0;
        double aPS = 0.0;
        double aSS = 0.0;
        addAFast(aPP, alpha1Abs2, m_kernel11Primary, m_kernel11Primary);
        addAFast(aPS, alpha1Abs2, m_kernel11Primary, m_kernel11Secondary);
        addAFast(aSS, alpha1Abs2, m_kernel11Secondary, m_kernel11Secondary);
        addAFast(aPP, alpha2Abs2, m_kernel22Primary, m_kernel22Primary);
        addAFast(aPS, alpha2Abs2, m_kernel22Primary, m_kernel22Secondary);
        addAFast(aSS, alpha2Abs2, m_kernel22Secondary, m_kernel22Secondary);

        const double linearPrimary = 0.5 * (m_lambda + mu) * uPrimary - mu * cPrimary;
        const double linearSecondary = 0.5 * (m_lambda + mu) * uSecondary - mu * cSecondary;
        double g11 = aPP + m_upsilon;
        double g12 = aPS + m_upsilon;
        double g22 = aSS + m_upsilon;
        const double r1 = (bPrimary - linearPrimary) + m_upsilon;
        const double r2 = (bSecondary - linearSecondary) + m_upsilon;
        double determinant = g11 * g22 - g12 * g12;
        cv::Vec2d coeff(previousPrimary, previousSecondary);
        if (std::abs(determinant) < 1e-12)
        {
            g11 += 1e-9;
            g22 += 1e-9;
            determinant = g11 * g22 - g12 * g12;
        }
        if (std::abs(determinant) > 1e-16)
        {
            coeff[0] = (g22 * r1 - g12 * r2) / determinant;
            coeff[1] = (-g12 * r1 + g11 * r2) / determinant;
        }
        coeff[0] = std::max(0.0, coeff[0]);
        coeff[1] = std::max(0.0, coeff[1]);
        const double coeffSum = coeff[0] + coeff[1];
        if (coeffSum > 1e-12)
        {
            m_kernelCoeffPrimary =
                static_cast<float>((1.0F - trainInterpFactor) * m_kernelCoeffPrimary
                    + trainInterpFactor * static_cast<float>(coeff[0] / coeffSum));
            m_kernelCoeffSecondary =
                static_cast<float>((1.0F - trainInterpFactor) * m_kernelCoeffSecondary
                    + trainInterpFactor * static_cast<float>(coeff[1] / coeffSum));
        }
        const float finalSum = m_kernelCoeffPrimary + m_kernelCoeffSecondary;
        if (finalSum > 1e-12F)
        {
            m_kernelCoeffPrimary /= finalSum;
            m_kernelCoeffSecondary /= finalSum;
        }
        else
        {
            m_kernelCoeffPrimary = previousPrimary;
            m_kernelCoeffSecondary = previousSecondary;
        }

        alphaDiffHistory.push_back(calcDiffFrobeniusNorm2(alphaTemplate1, alphaTemplate2));
        if (alphaDiffHistory.size() > 5)
        {
            alphaDiffHistory.pop_front();
        }
        if (alphaDiffHistory.size() == 5)
        {
            const double meanDiff = calcMean(alphaDiffHistory);
            const double relStd = calcStdDev(alphaDiffHistory) / (meanDiff + 1e-12);
            if (relStd < 1e-2)
            {
                break;
            }
        }
        mu *= muInc;
    }

    m_alphaF = alphaTemplate1.clone();
    m_histTmplPrimary =
        trainInterpFactor * currentPrimary + (1.0F - trainInterpFactor) * m_histTmplPrimary;
    m_histTmplSecondary =
        trainInterpFactor * currentSecondary + (1.0F - trainInterpFactor) * m_histTmplSecondary;
    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
}

void CKcfTracker::train3TMKCF(
    const cv::Mat &image,
    float trainInterpFactor,
    double mu,
    double muInc)
{
    if (m_imageSim.empty() || m_imageSimROI.width <= 0 || m_imageSimROI.height <= 0)
    {
        train2TMKCF(image, trainInterpFactor, mu, muInc);
        return;
    }

    const int featureHeight = std::max(1, m_alphaF.rows);
    const int featureWidth = std::max(1, m_alphaF.cols);
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    cv::Mat simPrimary;
    cv::Mat simSecondary;
    getFeatures(image, toIntegerRect(m_roi), 1.0F, currentPrimary, currentSecondary);
    getFeatures(m_imageSim, m_imageSimROI, 1.0F, simPrimary, simSecondary);
    if (m_histTmplPrimary.empty() || m_histTmplPrimary.size() != currentPrimary.size())
    {
        m_histTmplPrimary = currentPrimary.clone();
    }
    if (m_histTmplSecondary.empty() || m_histTmplSecondary.size() != currentSecondary.size())
    {
        m_histTmplSecondary = currentSecondary.clone();
    }
    if (currentPrimary.size() != simPrimary.size() || currentSecondary.size() != simSecondary.size())
    {
        train2TMKCF(image, trainInterpFactor, mu, muInc);
        return;
    }

    computeKernels(
        currentPrimary,
        currentSecondary,
        simPrimary,
        simSecondary,
        m_histTmplPrimary,
        m_histTmplSecondary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary,
        m_sigmaSecondary);

    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
    float previousPrimary = m_kernelCoeffPrimary;
    float previousSecondary = m_kernelCoeffSecondary;
    cv::Mat alphaTemplate1 = solveSingleTemplateAlpha(
        m_kernelCoeffPrimary,
        m_kernelCoeffSecondary,
        m_kernel11Primary,
        m_kernel11Secondary);
    cv::Mat alphaTemplate2 = solveSingleTemplateAlpha(
        m_kernelCoeffPrimary,
        m_kernelCoeffSecondary,
        m_kernel22Primary,
        m_kernel22Secondary);
    cv::Mat alphaTemplate3 = solveSingleTemplateAlpha(
        m_kernelCoeffPrimary,
        m_kernelCoeffSecondary,
        m_kernel33Primary,
        m_kernel33Secondary);

    std::deque<double> alphaDiffHistory;
    constexpr int kMaxOuterIterations = 1;
    for (int outer = 0; outer < kMaxOuterIterations; ++outer)
    {
        const cv::Mat kernel11 =
            m_kernel11Primary * m_kernelCoeffPrimary + m_kernel11Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel22 =
            m_kernel22Primary * m_kernelCoeffPrimary + m_kernel22Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel33 =
            m_kernel33Primary * m_kernelCoeffPrimary + m_kernel33Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel12 =
            m_kernel12Primary * m_kernelCoeffPrimary + m_kernel12Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel13 =
            m_kernel13Primary * m_kernelCoeffPrimary + m_kernel13Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel23 =
            m_kernel23Primary * m_kernelCoeffPrimary + m_kernel23Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel21 =
            m_kernel21Primary * m_kernelCoeffPrimary + m_kernel21Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel31 =
            m_kernel31Primary * m_kernelCoeffPrimary + m_kernel31Secondary * m_kernelCoeffSecondary;
        const cv::Mat kernel32 =
            m_kernel32Primary * m_kernelCoeffPrimary + m_kernel32Secondary * m_kernelCoeffSecondary;

        const float denomOffset = m_lambda + static_cast<float>(2.0 * mu);
        cv::Mat denom1 = addRealScalar(kernel11, denomOffset);
        cv::Mat denom2 = addRealScalar(kernel22, denomOffset);
        cv::Mat denom3 = addRealScalar(kernel33, denomOffset);
        alphaTemplate1 = KcfFftTools::complexDivision(
            complexAddition(
                m_prob,
                complexMultiplicationScalar(
                    complexAddition(
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel12, kernel11),
                            alphaTemplate2),
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel13, kernel11),
                            alphaTemplate3)),
                    static_cast<float>(mu))),
            denom1);
        alphaTemplate2 = KcfFftTools::complexDivision(
            complexAddition(
                m_prob,
                complexMultiplicationScalar(
                    complexAddition(
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel21, kernel22),
                            alphaTemplate1),
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel23, kernel22),
                            alphaTemplate3)),
                    static_cast<float>(mu))),
            denom2);
        alphaTemplate3 = KcfFftTools::complexDivision(
            complexAddition(
                m_prob,
                complexMultiplicationScalar(
                    complexAddition(
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel31, kernel33),
                            alphaTemplate1),
                        KcfFftTools::complexMultiplication(
                            KcfFftTools::complexDivision(kernel32, kernel33),
                            alphaTemplate2)),
                    static_cast<float>(mu))),
            denom3);

        cv::Mat alpha1Abs2;
        cv::Mat alpha2Abs2;
        cv::Mat alpha3Abs2;
        computeAbs2(alphaTemplate1, alpha1Abs2);
        computeAbs2(alphaTemplate2, alpha2Abs2);
        computeAbs2(alphaTemplate3, alpha3Abs2);
        const double uPrimary =
            sumAbs2MulRealK(alpha1Abs2, m_kernel11Primary)
            + sumAbs2MulRealK(alpha2Abs2, m_kernel22Primary)
            + sumAbs2MulRealK(alpha3Abs2, m_kernel33Primary);
        const double uSecondary =
            sumAbs2MulRealK(alpha1Abs2, m_kernel11Secondary)
            + sumAbs2MulRealK(alpha2Abs2, m_kernel22Secondary)
            + sumAbs2MulRealK(alpha3Abs2, m_kernel33Secondary);
        const double cPrimary =
            sumConjAMulKMulB(alphaTemplate1, m_kernel12Primary, alphaTemplate2)
            + sumConjAMulKMulB(alphaTemplate1, m_kernel13Primary, alphaTemplate3)
            + sumConjAMulKMulB(alphaTemplate2, m_kernel23Primary, alphaTemplate3);
        const double cSecondary =
            sumConjAMulKMulB(alphaTemplate1, m_kernel12Secondary, alphaTemplate2)
            + sumConjAMulKMulB(alphaTemplate1, m_kernel13Secondary, alphaTemplate3)
            + sumConjAMulKMulB(alphaTemplate2, m_kernel23Secondary, alphaTemplate3);
        const double bPrimary =
            sumConjAMulKMulProb(alphaTemplate1, m_kernel11Primary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate2, m_kernel22Primary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate3, m_kernel33Primary, m_prob);
        const double bSecondary =
            sumConjAMulKMulProb(alphaTemplate1, m_kernel11Secondary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate2, m_kernel22Secondary, m_prob)
            + sumConjAMulKMulProb(alphaTemplate3, m_kernel33Secondary, m_prob);

        double aPP = 0.0;
        double aPS = 0.0;
        double aSS = 0.0;
        addAFast(aPP, alpha1Abs2, m_kernel11Primary, m_kernel11Primary);
        addAFast(aPS, alpha1Abs2, m_kernel11Primary, m_kernel11Secondary);
        addAFast(aSS, alpha1Abs2, m_kernel11Secondary, m_kernel11Secondary);
        addAFast(aPP, alpha2Abs2, m_kernel22Primary, m_kernel22Primary);
        addAFast(aPS, alpha2Abs2, m_kernel22Primary, m_kernel22Secondary);
        addAFast(aSS, alpha2Abs2, m_kernel22Secondary, m_kernel22Secondary);
        addAFast(aPP, alpha3Abs2, m_kernel33Primary, m_kernel33Primary);
        addAFast(aPS, alpha3Abs2, m_kernel33Primary, m_kernel33Secondary);
        addAFast(aSS, alpha3Abs2, m_kernel33Secondary, m_kernel33Secondary);

        const double linearPrimary = 0.5 * (m_lambda + 2.0 * mu) * uPrimary - mu * cPrimary;
        const double linearSecondary = 0.5 * (m_lambda + 2.0 * mu) * uSecondary - mu * cSecondary;
        double g11 = aPP + m_upsilon;
        double g12 = aPS + m_upsilon;
        double g22 = aSS + m_upsilon;
        const double r1 = (bPrimary - linearPrimary) + m_upsilon;
        const double r2 = (bSecondary - linearSecondary) + m_upsilon;
        double determinant = g11 * g22 - g12 * g12;
        cv::Vec2d coeff(previousPrimary, previousSecondary);
        if (std::abs(determinant) < 1e-12)
        {
            g11 += 1e-9;
            g22 += 1e-9;
            determinant = g11 * g22 - g12 * g12;
        }
        if (std::abs(determinant) > 1e-16)
        {
            coeff[0] = (g22 * r1 - g12 * r2) / determinant;
            coeff[1] = (-g12 * r1 + g11 * r2) / determinant;
        }
        coeff[0] = std::max(0.0, coeff[0]);
        coeff[1] = std::max(0.0, coeff[1]);
        const double coeffSum = coeff[0] + coeff[1];
        if (coeffSum > 1e-12)
        {
            m_kernelCoeffPrimary =
                static_cast<float>((1.0F - trainInterpFactor) * m_kernelCoeffPrimary
                    + trainInterpFactor * static_cast<float>(coeff[0] / coeffSum));
            m_kernelCoeffSecondary =
                static_cast<float>((1.0F - trainInterpFactor) * m_kernelCoeffSecondary
                    + trainInterpFactor * static_cast<float>(coeff[1] / coeffSum));
        }
        const float finalSum = m_kernelCoeffPrimary + m_kernelCoeffSecondary;
        if (finalSum > 1e-12F)
        {
            m_kernelCoeffPrimary /= finalSum;
            m_kernelCoeffSecondary /= finalSum;
        }
        else
        {
            m_kernelCoeffPrimary = previousPrimary;
            m_kernelCoeffSecondary = previousSecondary;
        }

        const double totalDiff =
            calcDiffFrobeniusNorm2(alphaTemplate1, alphaTemplate2)
            + calcDiffFrobeniusNorm2(alphaTemplate1, alphaTemplate3)
            + calcDiffFrobeniusNorm2(alphaTemplate2, alphaTemplate3);
        alphaDiffHistory.push_back(totalDiff);
        if (alphaDiffHistory.size() > 5)
        {
            alphaDiffHistory.pop_front();
        }
        if (alphaDiffHistory.size() == 5)
        {
            const double meanDiff = calcMean(alphaDiffHistory);
            const double relStd = calcStdDev(alphaDiffHistory) / (meanDiff + 1e-12);
            if (relStd < 0.01)
            {
                break;
            }
        }
        mu *= muInc;
    }

    m_alphaF = alphaTemplate1.clone();
    m_histTmplPrimary =
        trainInterpFactor * currentPrimary + (1.0F - trainInterpFactor) * m_histTmplPrimary;
    m_histTmplSecondary =
        trainInterpFactor * currentSecondary + (1.0F - trainInterpFactor) * m_histTmplSecondary;
    m_kernelCoeffPrimary = 1.0F;
    m_kernelCoeffSecondary = 0.0F;
}

cv::Mat CKcfTracker::solveSingleTemplateAlpha(
    float primaryCoeff,
    float secondaryCoeff,
    const cv::Mat &primaryKernel,
    const cv::Mat &secondaryKernel) const
{
    cv::Mat combinedKernel = primaryCoeff * primaryKernel + secondaryCoeff * secondaryKernel;
    cv::Mat denominator = addRealScalar(combinedKernel, m_lambda);
    denominator = addRealScalar(denominator, 1e-7F);
    return KcfFftTools::complexDivision(m_prob, denominator);
}

void CKcfTracker::computeKernels(
    const cv::Mat &currentPrimary,
    const cv::Mat &currentSecondary,
    const cv::Mat &referencePrimary,
    const cv::Mat &referenceSecondary,
    int height,
    int width,
    float primarySigma,
    float secondarySigma)
{
    m_kernel11Primary = KcfFftTools::fftd(gaussianCorrelation(
        currentPrimary,
        currentPrimary,
        height,
        width,
        primarySigma));
    m_kernel11Secondary = KcfFftTools::fftd(gaussianCorrelation(
        currentSecondary,
        currentSecondary,
        height,
        width,
        secondarySigma));
    m_kernel22Primary = KcfFftTools::fftd(gaussianCorrelation(
        referencePrimary,
        referencePrimary,
        height,
        width,
        primarySigma));
    m_kernel22Secondary = KcfFftTools::fftd(gaussianCorrelation(
        referenceSecondary,
        referenceSecondary,
        height,
        width,
        secondarySigma));
    m_kernel12Primary = KcfFftTools::fftd(gaussianCorrelation(
        currentPrimary,
        referencePrimary,
        height,
        width,
        primarySigma));
    m_kernel12Secondary = KcfFftTools::fftd(gaussianCorrelation(
        currentSecondary,
        referenceSecondary,
        height,
        width,
        secondarySigma));
    m_kernel21Primary = KcfFftTools::complexConj(m_kernel12Primary);
    m_kernel21Secondary = KcfFftTools::complexConj(m_kernel12Secondary);
}

void CKcfTracker::computeKernels(
    const cv::Mat &template1Primary,
    const cv::Mat &template1Secondary,
    const cv::Mat &template2Primary,
    const cv::Mat &template2Secondary,
    const cv::Mat &template3Primary,
    const cv::Mat &template3Secondary,
    int height,
    int width,
    float primarySigma,
    float secondarySigma)
{
    m_kernel11Primary = KcfFftTools::fftd(gaussianCorrelation(
        template1Primary,
        template1Primary,
        height,
        width,
        primarySigma));
    m_kernel11Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template1Secondary,
        template1Secondary,
        height,
        width,
        secondarySigma));
    m_kernel22Primary = KcfFftTools::fftd(gaussianCorrelation(
        template2Primary,
        template2Primary,
        height,
        width,
        primarySigma));
    m_kernel22Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template2Secondary,
        template2Secondary,
        height,
        width,
        secondarySigma));
    m_kernel33Primary = KcfFftTools::fftd(gaussianCorrelation(
        template3Primary,
        template3Primary,
        height,
        width,
        primarySigma));
    m_kernel33Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template3Secondary,
        template3Secondary,
        height,
        width,
        secondarySigma));
    m_kernel12Primary = KcfFftTools::fftd(gaussianCorrelation(
        template1Primary,
        template2Primary,
        height,
        width,
        primarySigma));
    m_kernel12Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template1Secondary,
        template2Secondary,
        height,
        width,
        secondarySigma));
    m_kernel13Primary = KcfFftTools::fftd(gaussianCorrelation(
        template1Primary,
        template3Primary,
        height,
        width,
        primarySigma));
    m_kernel13Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template1Secondary,
        template3Secondary,
        height,
        width,
        secondarySigma));
    m_kernel23Primary = KcfFftTools::fftd(gaussianCorrelation(
        template2Primary,
        template3Primary,
        height,
        width,
        primarySigma));
    m_kernel23Secondary = KcfFftTools::fftd(gaussianCorrelation(
        template2Secondary,
        template3Secondary,
        height,
        width,
        secondarySigma));
    m_kernel21Primary = KcfFftTools::complexConj(m_kernel12Primary);
    m_kernel21Secondary = KcfFftTools::complexConj(m_kernel12Secondary);
    m_kernel31Primary = KcfFftTools::complexConj(m_kernel13Primary);
    m_kernel31Secondary = KcfFftTools::complexConj(m_kernel13Secondary);
    m_kernel32Primary = KcfFftTools::complexConj(m_kernel23Primary);
    m_kernel32Secondary = KcfFftTools::complexConj(m_kernel23Secondary);
}

void CKcfTracker::normalizeImage(cv::Mat &image) const
{
    if (image.empty())
    {
        return;
    }
    if (image.depth() != CV_8U)
    {
        image.convertTo(image, CV_8U);
    }
}

void CKcfTracker::clampRoiToImage(const cv::Mat &image)
{
    if (image.empty())
    {
        return;
    }
    if (m_roi.width < 1.0F)
    {
        m_roi.width = 1.0F;
    }
    if (m_roi.height < 1.0F)
    {
        m_roi.height = 1.0F;
    }
    if (m_roi.x + m_roi.width <= 0.0F)
    {
        m_roi.x = -m_roi.width + 2.0F;
    }
    if (m_roi.y + m_roi.height <= 0.0F)
    {
        m_roi.y = -m_roi.height + 2.0F;
    }
    if (m_roi.x >= image.cols - 1)
    {
        m_roi.x = static_cast<float>(image.cols - 2);
    }
    if (m_roi.y >= image.rows - 1)
    {
        m_roi.y = static_cast<float>(image.rows - 2);
    }
}

cv::Mat CKcfTracker::extractPatchFeature(
    const cv::Mat &image,
    const cv::Rect_<float> &roi,
    float scaleAdjust) const
{
    return KcfRectTools::getGrayImage(extractPatchImage(image, roi, scaleAdjust));
}

void CKcfTracker::initializeTemplateGeometryFromRoi(const cv::Rect_<float> &roi)
{
    const int paddedWidth = std::max(2, static_cast<int>(roi.width * m_padding));
    const int paddedHeight = std::max(2, static_cast<int>(roi.height * m_padding));

    cv::Size targetSize;
    if (m_template_size > 1)
    {
        const double scale = std::max(paddedWidth, paddedHeight)
            / static_cast<double>(m_template_size);
        targetSize.width = std::max(2, static_cast<int>(paddedWidth / scale));
        targetSize.height = std::max(2, static_cast<int>(paddedHeight / scale));
    }
    else
    {
        targetSize.width = paddedWidth;
        targetSize.height = paddedHeight;
    }

    if (m_hogFeatures)
    {
        targetSize.width =
            ((targetSize.width / (2 * m_cell_size)) * 2 * m_cell_size) + 2 * m_cell_size;
        targetSize.height =
            ((targetSize.height / (2 * m_cell_size)) * 2 * m_cell_size) + 2 * m_cell_size;
    }
    else
    {
        targetSize.width = std::max(2, (targetSize.width / 2) * 2);
        targetSize.height = std::max(2, (targetSize.height / 2) * 2);
    }
    m_tmplSize = targetSize;
    m_scale =
        paddedWidth >= paddedHeight
            ? paddedWidth / static_cast<float>(targetSize.width)
            : paddedHeight / static_cast<float>(targetSize.height);
}

cv::Mat CKcfTracker::extractPatchImage(
    const cv::Mat &image,
    const cv::Rect_<float> &roi,
    float scaleAdjust) const
{
    const double cx = roi.x + roi.width * 0.5;
    const double cy = roi.y + roi.height * 0.5;
    const cv::Size targetSize(
        std::max(2, m_tmplSize.width),
        std::max(2, m_tmplSize.height));

    cv::Rect extractedRoi;
    extractedRoi.width = std::max(2, static_cast<int>(scaleAdjust * m_scale * targetSize.width));
    extractedRoi.height = std::max(2, static_cast<int>(scaleAdjust * m_scale * targetSize.height));
    extractedRoi.x = static_cast<int>(std::round(cx - extractedRoi.width * 0.5));
    extractedRoi.y = static_cast<int>(std::round(cy - extractedRoi.height * 0.5));

    cv::Mat patch = KcfRectTools::subwindow(image, extractedRoi, cv::BORDER_REPLICATE);
    if (patch.size() != targetSize)
    {
        cv::resize(patch, patch, targetSize);
    }
    return patch;
}

double CKcfTracker::normalizedObservationNoise(float peakValue)
{
    if (peakValue > m_peakValueMaximum)
    {
        m_peakValueMaximum = peakValue;
    }
    if (peakValue < m_peakValueMinimum)
    {
        m_peakValueMinimum = peakValue;
    }
    const float range = std::max(1e-6F, m_peakValueMaximum - m_peakValueMinimum);
    const float normalized = (peakValue - m_peakValueMinimum) / range;
    if (normalized > 0.2F)
    {
        return 1e-3 - normalized * (1e-3 - 5e-6);
    }
    return 1e-2 - normalized * (1e-2 - 5e-6);
}

} // namespace i2r_rlc::vision
