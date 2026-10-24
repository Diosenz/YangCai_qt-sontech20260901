/**
 * @file GapDisabledAdapter.cpp
 * @brief 无 PCL 离线图像回放的显式受限链接边界；误用间隙检测立即失败。
 * @details 不替代图像算法。运行器拒绝 enable_gap=true，异常是第二道保护。
 */
#include "feature/PointCloudGapWidthDetector.h"
#include <stdexcept>

namespace i2r_rlc::vision {
PointCloudGapWidthResult CPointCloudGapWidthDetector::detect(
    const WeldPointCloud &, const WeldSpacePoint &,
    const PointCloudGapWidthOptions &)
{
    throw std::runtime_error("PCL gap detection excluded from image-branch replay");
}
}
