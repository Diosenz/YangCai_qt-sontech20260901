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
#include <array>
#include <cmath>
#include <complex>
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
 * @brief 单次核矩阵构建中的频谱值快照，不跨帧、ROI、参考或会话缓存。
 * @details 相同输入的每通道 FFT 和平方范数只计算一次；相关累加顺序保持原实现。
 */
struct CKernelFeatureSpectrum {
    std::vector<cv::Mat> channels;
    double squaredNorm = 0.0;
};

CKernelFeatureSpectrum makeKernelFeatureSpectrum(const cv::Mat &features, int height)
{
    cv::Mat floatFeatures;
    features.convertTo(floatFeatures, CV_32F);
    CKernelFeatureSpectrum result;
    result.squaredNorm = cv::sum(floatFeatures.mul(floatFeatures))[0];
    result.channels.reserve(static_cast<std::size_t>(floatFeatures.rows));
    for (int channel = 0; channel < floatFeatures.rows; ++channel) {
        result.channels.push_back(KcfFftTools::fftd(floatFeatures.row(channel).reshape(1, height)));
    }
    return result;
}

/** @brief 复用本调用内频谱，保持原高斯核归一化、shift、精度与通道求和顺序。 */
cv::Mat gaussianCorrelationFromSpectra(const CKernelFeatureSpectrum &left,
    const CKernelFeatureSpectrum &right, int height, int width, float sigma)
{
    cv::Mat spectrumSum = cv::Mat::zeros(height, width, CV_32FC2);
    for (std::size_t channel = 0; channel < left.channels.size(); ++channel) {
        cv::Mat spectrum;
        cv::mulSpectrums(left.channels[channel], right.channels[channel], spectrum, 0, true);
        spectrumSum += spectrum;
    }
    cv::Mat correlation = KcfFftTools::fftd(spectrumSum, true);
    KcfFftTools::rearrange(correlation);
    cv::Mat realCorrelation = KcfFftTools::real(correlation);
    cv::Mat distance = static_cast<float>(left.squaredNorm + right.squaredNorm) - 2.0F * realCorrelation;
    distance /= static_cast<float>(height * width * std::max(1, static_cast<int>(left.channels.size())));
    cv::max(distance, 0.0F, distance);
    distance /= sigma * sigma;
    cv::exp(-distance, distance);
    return distance;
}

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
 * @brief 将中心表示的相关核还原到零位移原点，再计算循环核的特征值。
 * @details 中心移位在偶数格频域引入交替符号；直接用于 MKCF 正则项会把
 *          半正定核误当成不定矩阵。此转换只用于训练，检测仍保留中心响应。
 */
cv::Mat zeroShiftSpectrum(const cv::Mat &centeredSpatial)
{
    cv::Mat spatial = centeredSpatial.clone();
    KcfFftTools::rearrange(spatial);
    return KcfFftTools::fftd(spatial);
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
            sum += static_cast<double>(absPtr[col]) * static_cast<double>(mRe * pRe + mIm * pIm);
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

/** @brief 同一固定目标的 MKCF 核、标签和正则系数；全部频谱以零位移为原点。 */
struct MkcfOptimizationProblem
{
    std::vector<std::vector<cv::Mat>> primary;
    std::vector<std::vector<cv::Mat>> secondary;
    cv::Mat target;
    double lambda = 0.0;
    double coupling = 0.0;
    double sumPenalty = 0.0;
};

/** @brief 同次优化的非负软权重及各模板系数；不得与其他模板基底混用。 */
struct MkcfOptimizationModel
{
    cv::Vec2d weights;
    std::vector<cv::Mat> alphas;
};

/**
 * @brief 按论文式 3-31 递推实际检测模板，随后必须用此基底重新训练 alpha。
 * @details previous 仅来自上一已接纳模型；候选失败时不推进模板或历史状态。
 */
cv::Mat acceptedTemplateCandidate(const cv::Mat &observed, const cv::Mat &previous,
    float learningRate)
{
    if (previous.empty() || previous.size() != observed.size()) { return observed.clone(); }
    return (1.0F - learningRate) * previous + learningRate * observed;
}

/**
 * @brief 精确比较二维非负 QP 的原点、两轴和内部主动集。
 * @details 最小化 .5*t'H*t-r't；和为一只通过 H/r 内的软罚项表达。
 *          不截断无约束解，不归一化，不用任意 jitter 隐藏不定 Hessian。
 */
bool solveNonnegativeWeights(const cv::Vec3d &hessian, const cv::Vec2d &rhs,
    cv::Vec2d &weights)
{
    const double scale = std::max({std::abs(hessian[0]), std::abs(hessian[1]),
        std::abs(hessian[2]), std::numeric_limits<double>::min()});
    const cv::Vec3d h = hessian / scale;
    const cv::Vec2d r = rhs / scale;
    const double determinant = h[0] * h[2] - h[1] * h[1];
    if (!std::isfinite(scale) || !std::isfinite(h[0]) || !std::isfinite(h[1])
        || !std::isfinite(h[2]) || !std::isfinite(r[0]) || !std::isfinite(r[1])
        || h[0] < -1e-12 || h[2] < -1e-12 || determinant < -1e-12) {
        return false;
    }
    weights = {0.0, 0.0};
    double best = 0.0;
    const auto consider = [&](const cv::Vec2d &candidate) {
        if (!std::isfinite(candidate[0]) || !std::isfinite(candidate[1])
            || candidate[0] < 0.0 || candidate[1] < 0.0) { return; }
        const double value = 0.5 * (h[0] * candidate[0] * candidate[0]
            + 2.0 * h[1] * candidate[0] * candidate[1]
            + h[2] * candidate[1] * candidate[1]) - r.dot(candidate);
        if (value < best) { best = value; weights = candidate; }
    };
    if (h[0] > 1e-12) { consider({std::max(0.0, r[0] / h[0]), 0.0}); }
    if (h[2] > 1e-12) { consider({0.0, std::max(0.0, r[1] / h[2])}); }
    if (determinant > 1e-12) {
        consider({(h[2] * r[0] - h[1] * r[1]) / determinant,
            (h[0] * r[1] - h[1] * r[0]) / determinant});
    }
    return true;
}

using MkcfComplexT = std::complex<double>;

/**
 * @brief 直接求解最多三个未知数的复数系统；病态或秩亏时由调用方回退 SVD。
 * @details 带列主元的消元仅改变求解成本，不改变方程或给分母添加正则。
 */
bool solveSmallComplexSystem(const std::array<MkcfComplexT, 9> &equations,
    const std::array<MkcfComplexT, 3> &rhs, int count,
    std::array<MkcfComplexT, 3> &solution)
{
    if (count < 1 || count > 3) { return false; }
    auto matrix = equations;
    auto values = rhs;
    double largest = 0.0;
    for (const auto &value : equations) { largest = std::max(largest, std::abs(value)); }
    const double tolerance = std::numeric_limits<double>::epsilon() * count * largest;
    solution = {};
    for (int column = 0; column < count; ++column) {
        int pivot = column;
        for (int row = column + 1; row < count; ++row) {
            if (std::abs(matrix[3 * row + column]) > std::abs(matrix[3 * pivot + column])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[3 * pivot + column]) <= tolerance) { return false; }
        if (pivot != column) {
            for (int j = 0; j < count; ++j) {
                std::swap(matrix[3 * column + j], matrix[3 * pivot + j]);
            }
            std::swap(values[column], values[pivot]);
        }
        for (int row = column + 1; row < count; ++row) {
            const auto factor = matrix[3 * row + column] / matrix[3 * column + column];
            matrix[3 * row + column] = {};
            for (int j = column + 1; j < count; ++j) {
                matrix[3 * row + j] -= factor * matrix[3 * column + j];
            }
            values[row] -= factor * values[column];
        }
    }
    for (int row = count - 1; row >= 0; --row) {
        auto value = values[row];
        for (int j = row + 1; j < count; ++j) { value -= matrix[3 * row + j] * solution[j]; }
        solution[row] = value / matrix[3 * row + row];
    }
    return true;
}

/** @brief 以原 2n 实数等价系统的同一后向残差判据验收复数解。 */
bool validSmallSystemResidual(const std::array<MkcfComplexT, 9> &equations,
    const std::array<MkcfComplexT, 3> &rhs, int count,
    const std::array<MkcfComplexT, 3> &solution)
{
    if (count < 1 || count > 3) { return false; }
    double residual2 = 0.0;
    double matrixNorm2 = 0.0;
    double rhsNorm2 = 0.0;
    double solutionNorm2 = 0.0;
    for (int i = 0; i < count; ++i) {
        auto residual = -rhs[i];
        rhsNorm2 += std::norm(rhs[i]);
        solutionNorm2 += std::norm(solution[i]);
        for (int j = 0; j < count; ++j) {
            const auto value = equations[3 * i + j];
            residual += value * solution[j];
            matrixNorm2 += std::norm(value);
        }
        residual2 += std::norm(residual);
    }
    const double scale = std::sqrt(2.0 * matrixNorm2 * solutionNorm2) + std::sqrt(rhsNorm2);
    return std::isfinite(residual2) && std::isfinite(scale) && std::sqrt(residual2)
        <= 1e-8 * std::max(scale, std::numeric_limits<double>::min());
}

/**
 * @brief 在最终权重的原始驻点方程上直接求解各模板 alpha。
 * @details 对每个频点解最多 6x6 的实数等价系统：
 * [kii^2+(lambda+(n-1)mu)kii]beta_i-mu*sum(kij*beta_j)=kii*g。
 * 双精度小系统直接求解，病态/秩亏频点使用 SVD 最小范数解；
 * 避免 kij/kii 和固定 epsilon 复除的偏置。
 */
bool solveMkcfAlphas(const MkcfOptimizationProblem &problem, const cv::Vec2d &weights,
    std::vector<cv::Mat> &alphas)
{
    const int count = static_cast<int>(problem.primary.size());
    CV_Assert(count >= 1 && count <= 3);
    std::vector<std::vector<cv::Mat>> kernels(count, std::vector<cv::Mat>(count));
    alphas.resize(count);
    for (int i = 0; i < count; ++i) {
        alphas[i] = cv::Mat::zeros(problem.target.size(), CV_32FC2);
        for (int j = 0; j < count; ++j) {
            kernels[i][j] = problem.primary[i][j] * weights[0]
                + problem.secondary[i][j] * weights[1];
        }
    }
    const double regularizer = problem.lambda + (count - 1) * problem.coupling;
    std::vector<unsigned char> validRows(problem.target.rows, 1);
    // 各任务只读核矩阵、独占频率行与求解缓冲；保留原方程及残差验收。
    cv::parallel_for_(cv::Range(0, problem.target.rows), [&](const cv::Range &range) {
        cv::Mat equations(2 * count, 2 * count, CV_64F);
        cv::Mat rhs(2 * count, 1, CV_64F);
        cv::Mat solution;
        for (int row = range.start; row < range.end; ++row) {
            for (int col = 0; col < problem.target.cols; ++col) {
                std::array<MkcfComplexT, 9> complexEquations{};
                std::array<MkcfComplexT, 3> complexRhs{};
                std::array<MkcfComplexT, 3> complexSolution{};
                const auto target = problem.target.at<cv::Vec2f>(row, col);
                for (int i = 0; i < count; ++i) {
                    const double self = kernels[i][i].at<cv::Vec2f>(row, col)[0];
                    complexRhs[i] = {self * target[0], self * target[1]};
                    for (int j = 0; j < count; ++j) {
                        const auto kernel = kernels[i][j].at<cv::Vec2f>(row, col);
                        const double re = i == j ? self * (self + regularizer)
                            : -problem.coupling * kernel[0];
                        const double im = i == j ? 0.0 : -problem.coupling * kernel[1];
                        complexEquations[3 * i + j] = {re, im};
                    }
                }
                if (!solveSmallComplexSystem(complexEquations, complexRhs, count, complexSolution)
                    || !validSmallSystemResidual(complexEquations, complexRhs, count, complexSolution)) {
                    equations.setTo(0.0);
                    rhs.setTo(0.0);
                    for (int i = 0; i < count; ++i) {
                        rhs.at<double>(2 * i) = complexRhs[i].real();
                        rhs.at<double>(2 * i + 1) = complexRhs[i].imag();
                        for (int j = 0; j < count; ++j) {
                            const auto value = complexEquations[3 * i + j];
                            equations.at<double>(2 * i, 2 * j) = value.real();
                            equations.at<double>(2 * i, 2 * j + 1) = -value.imag();
                            equations.at<double>(2 * i + 1, 2 * j) = value.imag();
                            equations.at<double>(2 * i + 1, 2 * j + 1) = value.real();
                        }
                    }
                    if (!cv::solve(equations, rhs, solution, cv::DECOMP_SVD)
                        || !cv::checkRange(solution)) { validRows[row] = 0; return; }
                    for (int i = 0; i < count; ++i) {
                        complexSolution[i] = {solution.at<double>(2 * i), solution.at<double>(2 * i + 1)};
                    }
                }
                if (!validSmallSystemResidual(complexEquations, complexRhs, count, complexSolution)) {
                    validRows[row] = 0;
                    return;
                }
                for (int i = 0; i < count; ++i) {
                    alphas[i].at<cv::Vec2f>(row, col) = {
                        static_cast<float>(complexSolution[i].real()),
                        static_cast<float>(complexSolution[i].imag())};
                }
            }
        }
    });
    if (std::find(validRows.begin(), validRows.end(), 0) != validRows.end()) { return false; }
    for (const auto &alpha : alphas) {
        if (!cv::checkRange(alpha)) { return false; }
    }
    return true;
}

/** @brief 固定候选模板与 mu 交替求解同一论文目标；最后重解 alpha，实际模板递推由调用方完成。 */
bool optimizeMkcf(const MkcfOptimizationProblem &problem, MkcfOptimizationModel &model)
{
    const int count = static_cast<int>(problem.primary.size());
    const double inverseCount = 1.0 / static_cast<double>(problem.target.total());
    for (int iteration = 0; iteration < 10; ++iteration) {
        if (!solveMkcfAlphas(problem, model.weights, model.alphas)) { return false; }
        cv::Vec3d a(0.0, 0.0, 0.0);
        cv::Vec2d b(0.0, 0.0);
        cv::Vec2d u(0.0, 0.0);
        cv::Vec2d cross(0.0, 0.0);
        for (int i = 0; i < count; ++i) {
            cv::Mat abs2;
            computeAbs2(model.alphas[i], abs2);
            const auto &primary = problem.primary[i][i];
            const auto &secondary = problem.secondary[i][i];
            addAFast(a[0], abs2, primary, primary);
            addAFast(a[1], abs2, primary, secondary);
            addAFast(a[2], abs2, secondary, secondary);
            b[0] += sumConjAMulKMulB(problem.target, primary, model.alphas[i]);
            b[1] += sumConjAMulKMulB(problem.target, secondary, model.alphas[i]);
            u[0] += sumAbs2MulRealK(abs2, primary);
            u[1] += sumAbs2MulRealK(abs2, secondary);
            for (int j = i + 1; j < count; ++j) {
                cross[0] += sumConjAMulKMulB(model.alphas[i], problem.primary[i][j], model.alphas[j]);
                cross[1] += sumConjAMulKMulB(model.alphas[i], problem.secondary[i][j], model.alphas[j]);
            }
        }
        // forward DFT 未缩放，Parseval 的 1/N 使软罚强度不随特征格尺寸改变。
        a *= inverseCount;
        b *= inverseCount;
        u *= inverseCount;
        cross *= inverseCount;
        const cv::Vec2d linear = 0.5 * (problem.lambda + (count - 1) * problem.coupling) * u
            - problem.coupling * cross;
        cv::Vec2d next;
        if (!solveNonnegativeWeights(a + cv::Vec3d::all(problem.sumPenalty),
                b - linear + cv::Vec2d::all(problem.sumPenalty), next)) { return false; }
        const double change = cv::norm(next - model.weights) / (1.0 + cv::norm(model.weights));
        model.weights = next;
        if (change < 1e-5) { break; }
    }
    return solveMkcfAlphas(problem, model.weights, model.alphas);
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
    m_kernelCoeffPrimary = 0.9F;
    m_kernelCoeffSecondary = 0.1F;
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
    if (m_featureMode == KcfFeatureMode::Mkcf)
    {
        // 窄激光条纹缩到固定 96 像素窗后会不足一个像素，亚格响应可反向。
        // MKCF 保留原采样尺度，仅按既有 FHOG 几何对齐 cell 边界。
        m_template_size = 1;
    }
}

void CKcfTracker::init(const cv::Rect &roi, cv::Mat image)
{
    m_pendingOnlineFeatures.release();
    m_pendingOnlineSecondaryFeatures.release();
    m_pendingOnlineImage.release();
    m_trainedTemplateCount = 0;
    m_detectionTmplPrimary.release();
    m_detectionTmplSecondary.release();
    m_histTmplPrimary.release();
    m_histTmplSecondary.release();
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
    if (m_featureMode == KcfFeatureMode::Mkcf) {
        m_mkcfProb = zeroShiftSpectrum(KcfFftTools::real(KcfFftTools::fftd(m_prob, true)));
    }
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

void CKcfTracker::setOnlineObservationMode(bool enabled)
{
    if (enabled == m_onlineObservationMode) { return; }
    m_onlineObservationMode = enabled;
    m_pendingOnlineFeatures.release();
    m_pendingOnlineSecondaryFeatures.release();
    m_pendingOnlineImage.release();
}

void CKcfTracker::commitOnlineObservationTraining(bool accepted)
{
    if (!m_onlineObservationMode || m_pendingOnlineFeatures.empty()) { return; }
    if (accepted) {
        const cv::Rect2f currentRoi = m_roi;
        m_roi = m_pendingOnlineRoi;
        bool trained = true;
        if (m_featureMode == KcfFeatureMode::Mkcf) {
            if (m_numTmpl == 2) {
                trained = train2TMKCF(m_pendingOnlineImage, m_interp_factor, 1e-6, 2.0);
            } else {
                trained = train3TMKCF(m_pendingOnlineImage, m_interp_factor, 1e-6, 2.0);
            }
            if (trained) { trainStandardReference(m_pendingOnlineFeatures, m_interp_factor); }
        } else {
            train(m_pendingOnlineFeatures, m_interp_factor);
        }
        m_roi = currentRoi;
        m_updateDiagnostic.trainingCommitted = trained;
        m_updateDiagnostic.trainingRejected = !trained;
    }
    m_pendingOnlineFeatures.release();
    m_pendingOnlineSecondaryFeatures.release();
    m_pendingOnlineImage.release();
}

void CKcfTracker::commitOnlineSimulationTraining()
{
    if (!m_onlineObservationMode || m_featureMode != KcfFeatureMode::Mkcf) { return; }
    commitOnlineObservationTraining(false);
    if (!hasSimulatedTemplate()) { return; }
    const cv::Rect2f trackedRoi = m_roi;
    m_roi = m_imageSimROI;
    // U_i=U_i^s；不能继续把异常实拍以小学习率写入递推模板。
    const bool trained = m_numTmpl == 2
        ? train2TMKCF(m_imageSim, 1.0F, 1e-6, 2.0)
        : train3TMKCF(m_imageSim, 1.0F, 1e-6, 2.0);
    m_roi = trackedRoi;
    m_updateDiagnostic.trainingCommitted = trained;
    m_updateDiagnostic.trainingRejected = !trained;
    m_updateDiagnostic.trainingFromSimulation = trained;
}

void CKcfTracker::correctOnlineObservationRoi(const cv::Rect2f &roi)
{
    if (!m_onlineObservationMode || m_pendingOnlineImage.empty()
        || !std::isfinite(roi.x) || !std::isfinite(roi.y)
        || !std::isfinite(roi.width) || !std::isfinite(roi.height)
        || roi.width <= 0.0F || roi.height <= 0.0F) { return; }
    m_roi = roi;
    clampRoiToImage(m_pendingOnlineImage);
    m_pendingOnlineRoi = m_roi;
    if (m_featureMode == KcfFeatureMode::Mkcf) {
        getFeatures(m_pendingOnlineImage, m_roi, 1.0F,
            m_pendingOnlineFeatures, m_pendingOnlineSecondaryFeatures);
    } else {
        m_pendingOnlineFeatures = getFeatures(m_pendingOnlineImage, false, 1.0F);
    }
    m_updateDiagnostic.finalRoi = m_roi;
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
    m_pendingOnlineFeatures.release();
    m_pendingOnlineSecondaryFeatures.release();
    m_pendingOnlineImage.release();
    m_updateDiagnostic = {};
    m_updateDiagnostic.onlineObservationMode = m_onlineObservationMode;
    // 必须在检测和本帧延迟训练之前冻结来源，不能用新仿真输入冒充旧响应的模型。
    if (m_featureMode == KcfFeatureMode::Mkcf) {
        m_updateDiagnostic.responseTemplateCount = m_trainedTemplateCount;
        m_updateDiagnostic.responseHogWeight = m_kernelCoeffPrimary;
        m_updateDiagnostic.responseGrayWeight = m_kernelCoeffSecondary;
    }
    m_updateDiagnostic.fusionMode = m_fusionOptions.mode;
    m_updateDiagnostic.simulationWeight = m_fusionOptions.simulationWeight;
    normalizeImage(image);
    clampRoiToImage(image);

    const cv::Point2f previousTopLeft(m_roi.x, m_roi.y);
    float peakValue = 0.0F;

    // MKCF 的联合模型直接决定相关位置，不能再强制投回标准 HOG 输出。
    cv::Point2f offset = (m_featureMode == KcfFeatureMode::Mkcf)
        ? detectMKCF(image, peakValue)
        : detect(m_tmpl, getFeatures(image, false, 1.0F), peakValue);

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

    const cv::Rect2f fusedRoi = m_roi;
    if (m_onlineObservationMode) {
        // 融合只影响业务权重；真实相关滤波不能在仿真位置继续搜索和学习。
        m_roi = m_updateDiagnostic.beforeFusionRoi;
    }
    cv::Mat currentFeature;
    if (m_onlineObservationMode && m_featureMode == KcfFeatureMode::Mkcf) {
        getFeatures(image, m_roi, 1.0F, currentFeature, m_pendingOnlineSecondaryFeatures);
    } else {
        currentFeature = (m_featureMode == KcfFeatureMode::FusedHogGray)
            ? fusedFeat(image, 1.0F) : getFeatures(image, false, 1.0F);
    }
    if (m_onlineObservationMode) {
        m_pendingOnlineFeatures = currentFeature;
        if (m_featureMode == KcfFeatureMode::Mkcf) { m_pendingOnlineImage = image; }
        m_pendingOnlineRoi = m_roi;
        m_updateDiagnostic.trainingDeferred = true;
    }
    else if (m_featureMode == KcfFeatureMode::Mkcf)
    {
        if (m_numTmpl == 2)
        {
            train2TMKCF(image, m_interp_factor, 1e-6, 2.0);
        }
        else
        {
            train3TMKCF(image, m_interp_factor, 1e-6, 2.0);
        }
        trainStandardReference(currentFeature, m_interp_factor);
    }
    else
    {
        train(currentFeature, m_interp_factor);
    }

    m_updateDiagnostic.finalRoi = fusedRoi;
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
    const cv::Rect_<float> &roi,
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

    // getGrayImage 已归一化到 [0, 1]；重复除以 255 会消除灰度辨别力。
    cv::Mat gray = KcfRectTools::getGrayImage(patch);
    gray -= 0.5F;
    // FHOG 归一化删除外围一圈 cell；双核必须以相同空间范围和格距表达位移。
    const cv::Rect graySupport(m_cell_size, m_cell_size,
        width * m_cell_size, height * m_cell_size);
    cv::Mat grayResized;
    cv::resize(gray(graySupport), grayResized,
        cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
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
    getFeatures(image, m_roi, scaleAdjust, hogFeature, grayFeature);
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
    if (m_detectionTmplPrimary.empty() || m_detectionTmplSecondary.empty() || m_alphaF.empty())
    {
        peakResponse = 0.0F;
        return cv::Point2f(0.0F, 0.0F);
    }

    const int featureHeight = m_alphaF.rows;
    const int featureWidth = m_alphaF.cols;
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    getFeatures(image, m_roi, 1.0F, currentPrimary, currentSecondary);

    cv::Mat corrPrimary = gaussianCorrelation(
        currentPrimary,
        m_detectionTmplPrimary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary);
    cv::Mat corrSecondary = gaussianCorrelation(
        currentSecondary,
        m_detectionTmplSecondary,
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
    // 复用Weld_Merge的全图均值/标准差定义，对应论文3-29。
    cv::Scalar responseMean;
    cv::Scalar responseDeviation;
    cv::meanStdDev(response, responseMean, responseDeviation);
    m_updateDiagnostic.responsePsr = responseDeviation[0] > 1e-9
        ? (peakValue - responseMean[0]) / responseDeviation[0] : 0.0;

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

bool CKcfTracker::trainMKCF(const cv::Mat &image, float trainInterpFactor)
{
    const int featureHeight = m_alphaF.rows;
    const int featureWidth = m_alphaF.cols;

    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    getFeatures(image, m_roi, 1.0F, currentPrimary, currentSecondary);
    currentPrimary = acceptedTemplateCandidate(currentPrimary,
        m_detectionTmplPrimary, trainInterpFactor);
    currentSecondary = acceptedTemplateCandidate(currentSecondary,
        m_detectionTmplSecondary, trainInterpFactor);

    if (m_histTmplPrimary.empty() || m_histTmplPrimary.size() != currentPrimary.size())
    {
        m_histTmplPrimary = currentPrimary.clone();
    }
    if (m_histTmplSecondary.empty() || m_histTmplSecondary.size() != currentSecondary.size())
    {
        m_histTmplSecondary = currentSecondary.clone();
    }

    cv::Mat kernelPrimary = zeroShiftSpectrum(gaussianCorrelation(
        currentPrimary,
        currentPrimary,
        featureHeight,
        featureWidth,
        m_sigmaPrimary));
    cv::Mat kernelSecondary = zeroShiftSpectrum(gaussianCorrelation(
        currentSecondary,
        currentSecondary,
        featureHeight,
        featureWidth,
        m_sigmaSecondary));

    MkcfOptimizationProblem problem;
    problem.primary = {{kernelPrimary}};
    problem.secondary = {{kernelSecondary}};
    problem.target = m_mkcfProb;
    problem.lambda = m_lambda;
    problem.sumPenalty = m_upsilon;
    MkcfOptimizationModel model;
    model.weights = {m_kernelCoeffPrimary, m_kernelCoeffSecondary};
    if (!optimizeMkcf(problem, model)) { return false; }
    m_kernelCoeffPrimary = static_cast<float>(model.weights[0]);
    m_kernelCoeffSecondary = static_cast<float>(model.weights[1]);
    m_alphaF = model.alphas[0].clone();
    m_trainedTemplateCount = 1;
    m_detectionTmplPrimary = currentPrimary.clone();
    m_detectionTmplSecondary = currentSecondary.clone();
    m_histTmplPrimary = currentPrimary.clone();
    m_histTmplSecondary = currentSecondary.clone();
    return true;
}

bool CKcfTracker::train2TMKCF(
    const cv::Mat &image,
    float trainInterpFactor,
    double mu,
    double muInc)
{
    if (m_imageSim.empty() || m_imageSimROI.width <= 0 || m_imageSimROI.height <= 0)
    {
        return trainMKCF(image, trainInterpFactor);
    }

    const int featureHeight = std::max(1, m_alphaF.rows);
    const int featureWidth = std::max(1, m_alphaF.cols);
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    cv::Mat simPrimary;
    cv::Mat simSecondary;
    if (m_onlineObservationMode && image.data == m_pendingOnlineImage.data
        && m_roi == m_pendingOnlineRoi && !m_pendingOnlineSecondaryFeatures.empty()) {
        currentPrimary = m_pendingOnlineFeatures;
        currentSecondary = m_pendingOnlineSecondaryFeatures;
    } else {
        getFeatures(image, m_roi, 1.0F, currentPrimary, currentSecondary);
    }
    currentPrimary = acceptedTemplateCandidate(currentPrimary,
        m_detectionTmplPrimary, trainInterpFactor);
    currentSecondary = acceptedTemplateCandidate(currentSecondary,
        m_detectionTmplSecondary, trainInterpFactor);
    getFeatures(m_imageSim, m_imageSimROI, 1.0F, simPrimary, simSecondary);
    if (currentPrimary.size() != simPrimary.size() || currentSecondary.size() != simSecondary.size())
    {
        return trainMKCF(image, trainInterpFactor);
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

    // muInc 保留旧调用签名；优化过程中固定 mu，避免每轮改变目标。
    (void)muInc;
    MkcfOptimizationProblem problem;
    problem.primary = {{m_kernel11Primary, m_kernel12Primary},
        {m_kernel21Primary, m_kernel22Primary}};
    problem.secondary = {{m_kernel11Secondary, m_kernel12Secondary},
        {m_kernel21Secondary, m_kernel22Secondary}};
    problem.target = m_mkcfProb;
    problem.lambda = m_lambda;
    problem.coupling = mu;
    problem.sumPenalty = m_upsilon;
    MkcfOptimizationModel model;
    model.weights = {m_kernelCoeffPrimary, m_kernelCoeffSecondary};
    if (!optimizeMkcf(problem, model)) { return false; }
    m_kernelCoeffPrimary = static_cast<float>(model.weights[0]);
    m_kernelCoeffSecondary = static_cast<float>(model.weights[1]);
    m_alphaF = model.alphas[0].clone();
    m_trainedTemplateCount = 2;
    m_detectionTmplPrimary = currentPrimary.clone();
    m_detectionTmplSecondary = currentSecondary.clone();
    m_histTmplPrimary = currentPrimary.clone();
    m_histTmplSecondary = currentSecondary.clone();
    return true;
}

bool CKcfTracker::train3TMKCF(
    const cv::Mat &image,
    float trainInterpFactor,
    double mu,
    double muInc)
{
    if (m_imageSim.empty() || m_imageSimROI.width <= 0 || m_imageSimROI.height <= 0)
    {
        return train2TMKCF(image, trainInterpFactor, mu, muInc);
    }

    const int featureHeight = std::max(1, m_alphaF.rows);
    const int featureWidth = std::max(1, m_alphaF.cols);
    cv::Mat currentPrimary;
    cv::Mat currentSecondary;
    cv::Mat simPrimary;
    cv::Mat simSecondary;
    if (m_onlineObservationMode && image.data == m_pendingOnlineImage.data
        && m_roi == m_pendingOnlineRoi && !m_pendingOnlineSecondaryFeatures.empty()) {
        currentPrimary = m_pendingOnlineFeatures;
        currentSecondary = m_pendingOnlineSecondaryFeatures;
    } else {
        getFeatures(image, m_roi, 1.0F, currentPrimary, currentSecondary);
    }
    currentPrimary = acceptedTemplateCandidate(currentPrimary,
        m_detectionTmplPrimary, trainInterpFactor);
    currentSecondary = acceptedTemplateCandidate(currentSecondary,
        m_detectionTmplSecondary, trainInterpFactor);
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
        return train2TMKCF(image, trainInterpFactor, mu, muInc);
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

    // muInc 保留旧调用签名；三模板使用同一固定耦合目标完成交替最小化。
    (void)muInc;
    MkcfOptimizationProblem problem;
    problem.primary = {{m_kernel11Primary, m_kernel12Primary, m_kernel13Primary},
        {m_kernel21Primary, m_kernel22Primary, m_kernel23Primary},
        {m_kernel31Primary, m_kernel32Primary, m_kernel33Primary}};
    problem.secondary = {{m_kernel11Secondary, m_kernel12Secondary, m_kernel13Secondary},
        {m_kernel21Secondary, m_kernel22Secondary, m_kernel23Secondary},
        {m_kernel31Secondary, m_kernel32Secondary, m_kernel33Secondary}};
    problem.target = m_mkcfProb;
    problem.lambda = m_lambda;
    problem.coupling = mu;
    problem.sumPenalty = m_upsilon;
    MkcfOptimizationModel model;
    model.weights = {m_kernelCoeffPrimary, m_kernelCoeffSecondary};
    if (!optimizeMkcf(problem, model)) { return false; }
    m_kernelCoeffPrimary = static_cast<float>(model.weights[0]);
    m_kernelCoeffSecondary = static_cast<float>(model.weights[1]);
    m_alphaF = model.alphas[0].clone();
    m_trainedTemplateCount = 3;
    m_detectionTmplPrimary = currentPrimary.clone();
    m_detectionTmplSecondary = currentSecondary.clone();
    m_histTmplPrimary = currentPrimary.clone();
    m_histTmplSecondary = currentSecondary.clone();
    return true;
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
    m_kernel11Primary = zeroShiftSpectrum(gaussianCorrelation(
        currentPrimary,
        currentPrimary,
        height,
        width,
        primarySigma));
    m_kernel11Secondary = zeroShiftSpectrum(gaussianCorrelation(
        currentSecondary,
        currentSecondary,
        height,
        width,
        secondarySigma));
    m_kernel22Primary = zeroShiftSpectrum(gaussianCorrelation(
        referencePrimary,
        referencePrimary,
        height,
        width,
        primarySigma));
    m_kernel22Secondary = zeroShiftSpectrum(gaussianCorrelation(
        referenceSecondary,
        referenceSecondary,
        height,
        width,
        secondarySigma));
    m_kernel12Primary = zeroShiftSpectrum(gaussianCorrelation(
        currentPrimary,
        referencePrimary,
        height,
        width,
        primarySigma));
    m_kernel12Secondary = zeroShiftSpectrum(gaussianCorrelation(
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
    // 一次训练内六个特征各做一次正变换；所有状态随本调用结束释放。
    // 只有生产已确认尺寸一致的特征采用复用；异常输入沿用原核函数的检查。
    const bool canReuse = !template1Primary.empty() && !template1Secondary.empty()
        && template1Primary.cols == height * width && template1Secondary.cols == height * width
        && template1Primary.size() == template2Primary.size()
        && template1Primary.size() == template3Primary.size()
        && template1Secondary.size() == template2Secondary.size()
        && template1Secondary.size() == template3Secondary.size();
    std::array<CKernelFeatureSpectrum, 3> primarySpectra;
    std::array<CKernelFeatureSpectrum, 3> secondarySpectra;
    if (canReuse) {
        primarySpectra = {makeKernelFeatureSpectrum(template1Primary, height),
            makeKernelFeatureSpectrum(template2Primary, height),
            makeKernelFeatureSpectrum(template3Primary, height)};
        secondarySpectra = {makeKernelFeatureSpectrum(template1Secondary, height),
            makeKernelFeatureSpectrum(template2Secondary, height),
            makeKernelFeatureSpectrum(template3Secondary, height)};
    }
    const std::array<cv::Mat, 3> primary = {template1Primary, template2Primary, template3Primary};
    const std::array<cv::Mat, 3> secondary = {template1Secondary, template2Secondary, template3Secondary};
    const auto kernel = [&](bool isPrimary, int left, int right) {
        const auto sigma = isPrimary ? primarySigma : secondarySigma;
        const auto &spectra = isPrimary ? primarySpectra : secondarySpectra;
        const auto &features = isPrimary ? primary : secondary;
        return zeroShiftSpectrum(canReuse
            ? gaussianCorrelationFromSpectra(spectra[left], spectra[right], height, width, sigma)
            : gaussianCorrelation(features[left], features[right], height, width, sigma));
    };
    m_kernel11Primary = kernel(true, 0, 0);
    m_kernel11Secondary = kernel(false, 0, 0);
    m_kernel22Primary = kernel(true, 1, 1);
    m_kernel22Secondary = kernel(false, 1, 1);
    m_kernel33Primary = kernel(true, 2, 2);
    m_kernel33Secondary = kernel(false, 2, 2);
    m_kernel12Primary = kernel(true, 0, 1);
    m_kernel12Secondary = kernel(false, 0, 1);
    m_kernel13Primary = kernel(true, 0, 2);
    m_kernel13Secondary = kernel(false, 0, 2);
    m_kernel23Primary = kernel(true, 1, 2);
    m_kernel23Secondary = kernel(false, 1, 2);
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
    if (m_featureMode == KcfFeatureMode::Mkcf) {
        // 当前模板与下一帧搜索必须共用浮点原点；整数裁图残差不能随帧写回位移。
        cv::Mat mapX(targetSize, CV_32F);
        cv::Mat mapY(targetSize, CV_32F);
        const double stepX = extractedRoi.width / static_cast<double>(targetSize.width);
        const double stepY = extractedRoi.height / static_cast<double>(targetSize.height);
        for (int row = 0; row < targetSize.height; ++row) {
            float *x = mapX.ptr<float>(row);
            float *y = mapY.ptr<float>(row);
            for (int column = 0; column < targetSize.width; ++column) {
                x[column] = static_cast<float>(cx - extractedRoi.width * 0.5
                    + (column + 0.5) * stepX - 0.5);
                y[column] = static_cast<float>(cy - extractedRoi.height * 0.5
                    + (row + 0.5) * stepY - 0.5);
            }
        }
        cv::Mat patch;
        cv::remap(image, patch, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        return patch;
    }
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
