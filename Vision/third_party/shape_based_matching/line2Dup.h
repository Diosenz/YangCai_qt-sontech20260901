/*
 * shape_based_matching is distributed under the BSD 2-Clause License.
 * See LICENSE in this directory and THIRD_PARTY_NOTICES.md in the project root.
 */

/**
 * @file line2Dup.h
 * @brief shape_based_matching 的 LINE2D 形状匹配接口。
 * @details 该文件来自第三方 shape_based_matching，上游授权为 BSD 2-Clause。
 *          当前工程仅补充许可证可见性、Doxygen 注释、显式 include 和包含顺序，
 *          不改变模板生成、匹配或序列化行为。
 */

#ifndef CXXLINEMOD_H
#define CXXLINEMOD_H

#include <cassert>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "mipp.h"  // for SIMD in different platforms

namespace line2Dup
{

/**
 * @brief LINE2D 模板中的单个量化梯度特征点。
 *
 * `x/y` 为模板局部坐标，单位 pixel；`label` 为量化方向标签；`theta`
 * 为原始方向角缓存，单位由 OpenCV 梯度计算保持为 rad。
 */
struct Feature
{
    /// @brief 特征点 x 坐标，单位 pixel。
    int x;
    /// @brief 特征点 y 坐标，单位 pixel。
    int y;
    /// @brief 量化方向标签。
    int label;
    /// @brief 原始梯度方向角。
    float theta;

    /**
     * @brief 从 OpenCV FileStorage 节点读取特征。
     * @param fn 输入 YAML/XML 节点。
     */
    void read(const cv::FileNode &fn);

    /**
     * @brief 将特征写入 OpenCV FileStorage。
     * @param fs 输出 YAML/XML 存储对象。
     */
    void write(cv::FileStorage &fs) const;

    /// @brief 构造零坐标、零标签特征。
    Feature() : x(0), y(0), label(0) {}
    /**
     * @brief 构造指定坐标和方向标签的特征。
     * @param x 特征 x 坐标，单位 pixel。
     * @param y 特征 y 坐标，单位 pixel。
     * @param label 量化方向标签。
     */
    Feature(int x, int y, int label);
};
inline Feature::Feature(int _x, int _y, int _label) : x(_x), y(_y), label(_label) {}

/**
 * @brief 单层金字塔模板。
 *
 * `tl_x/tl_y` 记录模板在原图坐标系中的左上角，单位 pixel；`features`
 * 坐标为模板局部坐标。
 */
struct Template
{
    /// @brief 模板宽度，单位 pixel。
    int width;
    /// @brief 模板高度，单位 pixel。
    int height;
    /// @brief 模板左上角 x 坐标，单位 pixel。
    int tl_x;
    /// @brief 模板左上角 y 坐标，单位 pixel。
    int tl_y;
    /// @brief 所属图像金字塔层级，0 为原始分辨率。
    int pyramid_level;
    /// @brief 模板特征点集合。
    std::vector<Feature> features;

    /**
     * @brief 从 OpenCV FileStorage 节点读取模板。
     * @param fn 输入 YAML/XML 节点。
     */
    void read(const cv::FileNode &fn);

    /**
     * @brief 将模板写入 OpenCV FileStorage。
     * @param fs 输出 YAML/XML 存储对象。
     */
    void write(cv::FileStorage &fs) const;
};

/**
 * @brief 单层颜色梯度金字塔处理器。
 *
 * 对输入图像和 mask 计算梯度方向、幅值和候选特征，并在需要时向下一层降采样。
 */
class ColorGradientPyramid
{
public:
    /**
     * @brief 构造单层颜色梯度金字塔。
     * @param src 输入图像。
     * @param mask 有效区域 mask，非零像素参与特征提取。
     * @param weak_threshold 弱梯度阈值。
     * @param num_features 期望保留的特征数量。
     * @param strong_threshold 强梯度阈值。
     */
    ColorGradientPyramid(const cv::Mat &src, const cv::Mat &mask,
                                             float weak_threshold, size_t num_features,
                                             float strong_threshold);

    /**
     * @brief 将梯度方向量化为 LINE2D 响应图。
     * @param dst 输出量化图像。
     */
    void quantize(cv::Mat &dst) const;

    /**
     * @brief 从当前金字塔层提取模板。
     * @param templ 输出模板。
     * @return `true` 表示特征数量满足要求。
     */
    bool extractTemplate(Template &templ) const;

    /// @brief 将当前图像和 mask 下采样到下一层金字塔。
    void pyrDown();

public:
    /// @brief 重新计算当前层梯度、幅值和方向缓存。
    void update();
    /// @brief 带评分的候选特征。
    struct Candidate
    {
        /**
         * @brief 构造候选特征。
         * @param x x 坐标，单位 pixel。
         * @param y y 坐标，单位 pixel。
         * @param label 量化方向标签。
         * @param score 候选评分，越大越优先。
         */
        Candidate(int x, int y, int label, float score);

        /// @brief 按评分从高到低排序候选。
        bool operator<(const Candidate &rhs) const
        {
            return score > rhs.score;
        }

        /// @brief 候选特征点。
        Feature f;
        /// @brief 候选评分。
        float score;
    };

    /// @brief 当前层源图像。
    cv::Mat src;
    /// @brief 当前层有效区域 mask。
    cv::Mat mask;

    /// @brief 当前金字塔层级。
    int pyramid_level;
    /// @brief 量化后的梯度方向标签图。
    cv::Mat angle;
    /// @brief 梯度幅值图。
    cv::Mat magnitude;
    /// @brief 原始梯度方向角图。
    cv::Mat angle_ori;

    /// @brief 弱梯度阈值。
    float weak_threshold;
    /// @brief 每个模板最多保留的特征数。
    size_t num_features;
    /// @brief 强梯度阈值。
    float strong_threshold;
    /**
     * @brief 从候选点中选择空间上分散的特征。
     * @param candidates 候选特征，通常已按评分排序。
     * @param features 输出特征集合。
     * @param num_features 期望特征数量。
     * @param distance 最小特征间距，单位 pixel。
     * @return `true` 表示选中特征数量满足要求。
     */
    static bool selectScatteredFeatures(const std::vector<Candidate> &candidates,
                                                                            std::vector<Feature> &features,
                                                                            size_t num_features, float distance);
};
inline ColorGradientPyramid::Candidate::Candidate(int x, int y, int label, float _score) : f(x, y, label), score(_score) {}

/**
 * @brief LINE2D 颜色梯度 modality。
 */
class ColorGradient
{
public:
    /// @brief 构造默认阈值的颜色梯度 modality。
    ColorGradient();
    /**
     * @brief 构造指定阈值的颜色梯度 modality。
     * @param weak_threshold 弱梯度阈值。
     * @param num_features 模板特征数量。
     * @param strong_threshold 强梯度阈值。
     */
    ColorGradient(float weak_threshold, size_t num_features, float strong_threshold);

    /**
     * @brief 返回 modality 名称。
     * @return 固定名称字符串。
     */
    std::string name() const;

    /// @brief 弱梯度阈值。
    float weak_threshold;
    /// @brief 模板特征数量。
    size_t num_features;
    /// @brief 强梯度阈值。
    float strong_threshold;
    /// @brief 从 FileStorage 节点读取参数。
    void read(const cv::FileNode &fn);
    /// @brief 写入参数到 FileStorage。
    void write(cv::FileStorage &fs) const;

    /**
     * @brief 为输入图像创建颜色梯度金字塔首层。
     * @param src 输入图像。
     * @param mask 有效区域 mask。
     * @return OpenCV 智能指针持有的金字塔对象。
     */
    cv::Ptr<ColorGradientPyramid> process(const cv::Mat src, const cv::Mat &mask = cv::Mat()) const
    {
        return cv::makePtr<ColorGradientPyramid>(src, mask, weak_threshold, num_features, strong_threshold);
    }
};

/**
 * @brief 单个匹配结果。
 *
 * `x/y` 为匹配位置左上角坐标，单位 pixel；`similarity` 为相似度评分。
 */
struct Match
{
    /// @brief 构造默认空匹配。
    Match()
    {
    }

    /**
     * @brief 构造匹配结果。
     * @param x 匹配位置 x 坐标，单位 pixel。
     * @param y 匹配位置 y 坐标，单位 pixel。
     * @param similarity 相似度评分。
     * @param class_id 类别 ID。
     * @param template_id 模板 ID。
     */
    Match(int x, int y, float similarity, const std::string &class_id, int template_id);

    /// @brief 按相似度从高到低排序，相同相似度按模板 ID 去重稳定排序。
    bool operator<(const Match &rhs) const
    {
        // Secondarily sort on template_id for the sake of duplicate removal
        if (similarity != rhs.similarity)
            return similarity > rhs.similarity;
        else
            return template_id < rhs.template_id;
    }

    /// @brief 判断两个匹配是否位于同一位置、同一类别且相似度相同。
    bool operator==(const Match &rhs) const
    {
        return x == rhs.x && y == rhs.y && similarity == rhs.similarity && class_id == rhs.class_id;
    }

    /// @brief 匹配位置 x 坐标，单位 pixel。
    int x;
    /// @brief 匹配位置 y 坐标，单位 pixel。
    int y;
    /// @brief 相似度评分。
    float similarity;
    /// @brief 类别 ID。
    std::string class_id;
    /// @brief 模板 ID。
    int template_id;
};

inline Match::Match(int _x, int _y, float _similarity, const std::string &_class_id, int _template_id)
        : x(_x), y(_y), similarity(_similarity), class_id(_class_id), template_id(_template_id)
{
}

/**
 * @brief LINE2D 模板检测器。
 *
 * 管理类别、模板金字塔、线性内存响应图和序列化。对象不是线程安全的；
 * 并发匹配时应为每个线程使用独立 Detector 或在外部加锁。
 */
class Detector
{
public:
    /**
     * @brief 空构造，通常配合 read/readClasses 反序列化。
     */
    Detector();

    /**
     * @brief 构造指定金字塔阈值的检测器。
     * @param T 每层金字塔的量化响应阈值。
     */
    Detector(std::vector<int> T);

    /**
     * @brief 构造指定特征数、阈值和金字塔响应阈值的检测器。
     * @param num_features 每个模板保留的特征数量。
     * @param T 每层金字塔的量化响应阈值。
     * @param weak_thresh 弱梯度阈值。
     * @param strong_thresh 强梯度阈值。
     */
    Detector(int num_features, std::vector<int> T, float weak_thresh = 30.0f, float strong_thresh = 60.0f);

    /**
     * @brief 在输入图像上执行模板匹配。
     * @param sources 输入图像。
     * @param threshold 最小相似度阈值。
     * @param class_ids 可选类别过滤；为空表示匹配全部类别。
     * @param masks 可选有效区域 mask。
     * @return 按相似度排序并去重后的匹配结果。
     */
    std::vector<Match> match(cv::Mat sources, float threshold,
                                                     const std::vector<std::string> &class_ids = std::vector<std::string>(),
                                                     const cv::Mat masks = cv::Mat()) const;

    /**
     * @brief 从图像和 mask 添加一个模板。
     * @param sources 输入模板图像。
     * @param class_id 类别 ID。
     * @param object_mask 模板有效区域 mask。
     * @param num_features 可选特征数量；0 表示使用 modality 默认值。
     * @return 新模板 ID；失败时返回负值。
     */
    int addTemplate(const cv::Mat sources, const std::string &class_id,
                                    const cv::Mat &object_mask, int num_features = 0);

    /**
     * @brief 基于已有模板添加旋转模板。
     * @param class_id 类别 ID。
     * @param zero_id 基准模板 ID。
     * @param theta 旋转角度，单位 deg。
     * @param center 旋转中心，单位 pixel。
     * @return 新模板 ID；失败时返回负值。
     */
    int addTemplate_rotate(const std::string &class_id, int zero_id, float theta, cv::Point2f center);

    /// @brief 返回颜色梯度 modality。
    const cv::Ptr<ColorGradient> &getModalities() const { return modality; }

    /// @brief 返回指定金字塔层的响应阈值。
    int getT(int pyramid_level) const { return T_at_level[pyramid_level]; }

    /// @brief 返回金字塔层数。
    int pyramidLevels() const { return pyramid_levels; }

    /**
     * @brief 获取指定类别和模板 ID 的模板金字塔。
     * @param class_id 类别 ID。
     * @param template_id 模板 ID。
     * @return 模板金字塔引用。
     */
    const std::vector<Template> &getTemplates(const std::string &class_id, int template_id) const;

    /// @brief 返回所有类别的模板总数。
    int numTemplates() const;
    /// @brief 返回指定类别的模板数量。
    int numTemplates(const std::string &class_id) const;
    /// @brief 返回类别数量。
    int numClasses() const { return static_cast<int>(class_templates.size()); }

    /// @brief 返回全部类别 ID。
    std::vector<std::string> classIds() const;

    /// @brief 从 FileStorage 节点读取完整 Detector。
    void read(const cv::FileNode &fn);
    /// @brief 将完整 Detector 写入 FileStorage。
    void write(cv::FileStorage &fs) const;

    /**
     * @brief 读取单个类别模板。
     * @param fn 输入 YAML/XML 节点。
     * @param class_id_override 非空时覆盖文件中的类别 ID。
     * @return 实际读取的类别 ID。
     */
    std::string readClass(const cv::FileNode &fn, const std::string &class_id_override = "");
    /**
     * @brief 写出单个类别模板。
     * @param class_id 类别 ID。
     * @param fs 输出 FileStorage。
     */
    void writeClass(const std::string &class_id, cv::FileStorage &fs) const;

    /**
     * @brief 按格式批量读取类别模板文件。
     * @param class_ids 类别 ID 列表。
     * @param format 文件名格式，使用 `%s` 替换类别 ID。
     */
    void readClasses(const std::vector<std::string> &class_ids,
                                     const std::string &format = "templates_%s.yml.gz");
    /**
     * @brief 按格式批量写出类别模板文件。
     * @param format 文件名格式，使用 `%s` 替换类别 ID。
     */
    void writeClasses(const std::string &format = "templates_%s.yml.gz") const;

protected:
    /// @brief 颜色梯度 modality。
    cv::Ptr<ColorGradient> modality;
    /// @brief 金字塔层数。
    int pyramid_levels;
    /// @brief 每层金字塔的响应阈值。
    std::vector<int> T_at_level;

    /// @brief 单个模板的多层金字塔。
    typedef std::vector<Template> TemplatePyramid;
    /// @brief 类别到模板金字塔集合的映射。
    typedef std::map<std::string, std::vector<TemplatePyramid>> TemplatesMap;
    /// @brief 所有类别模板缓存。
    TemplatesMap class_templates;

    /// @brief 单层线性化响应图集合。
    typedef std::vector<cv::Mat> LinearMemories;
    /// @brief 线性内存金字塔，索引为 `[pyramid level][ColorGradient][quantized label]`。
    typedef std::vector<std::vector<LinearMemories>> LinearMemoryPyramid;

    /**
     * @brief 匹配单个类别的模板集合。
     * @param lm_pyramid 量化响应线性内存金字塔。
     * @param sizes 每层图像尺寸，单位 pixel。
     * @param threshold 最小相似度阈值。
     * @param matches 输出匹配结果。
     * @param class_id 类别 ID。
     * @param template_pyramids 该类别的模板金字塔集合。
     */
    void matchClass(const LinearMemoryPyramid &lm_pyramid,
                                    const std::vector<cv::Size> &sizes,
                                    float threshold, std::vector<Match> &matches,
                                    const std::string &class_id,
                                    const std::vector<TemplatePyramid> &template_pyramids) const;
};

} // namespace line2Dup

namespace shape_based_matching {
/**
 * @brief 生成旋转/尺度模板组合的辅助器。
 *
 * 输入图像和 mask 会按 `angle_range/scale_range` 组合生成模板源图。
 * 角度单位为 deg，尺度为无量纲比例。
 */
class shapeInfo_producer{
public:
    /// @brief 原始模板图像。
    cv::Mat src;
    /// @brief 原始模板 mask，非零像素表示有效区域。
    cv::Mat mask;

    /// @brief 角度范围，单位 deg；长度 0/1/2 分别表示默认/单值/闭区间。
    std::vector<float> angle_range;
    /// @brief 尺度范围；长度 0/1/2 分别表示默认/单值/闭区间。
    std::vector<float> scale_range;

    /// @brief 角度采样步长，单位 deg。
    float angle_step = 15;
    /// @brief 尺度采样步长。
    float scale_step = 0.5;
    /// @brief 闭区间浮点比较容差。
    float eps = 0.00001f;

    /// @brief 单个旋转/尺度组合。
    class Info{
    public:
        /// @brief 旋转角度，单位 deg。
        float angle;
        /// @brief 缩放比例。
        float scale;

        /// @brief 构造旋转/尺度组合。
        Info(float angle_, float scale_){
            angle = angle_;
            scale = scale_;
        }
    };
    /// @brief 已生成的旋转/尺度组合列表。
    std::vector<Info> infos;

    /**
     * @brief 构造信息生成器。
     * @param src 原始模板图像。
     * @param mask 可选有效区域 mask；为空时生成全 255 mask。
     */
    shapeInfo_producer(cv::Mat src, cv::Mat mask = cv::Mat()){
        this->src = src;
        if(mask.empty()){
            // make sure we have masks
            this->mask = cv::Mat(src.size(), CV_8UC1, {255});
        }else{
            this->mask = mask;
        }
    }

    /**
     * @brief 对图像执行绕中心旋转和缩放。
     * @param src 输入图像。
     * @param angle 旋转角度，单位 deg。
     * @param scale 缩放比例。
     * @return 变换后的图像，尺寸与输入相同。
     */
    static cv::Mat transform(cv::Mat src, float angle, float scale){
        cv::Mat dst;

        cv::Point2f center(src.cols/2.0f, src.rows/2.0f);
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, scale);
        cv::warpAffine(src, dst, rot_mat, src.size());

        return dst;
    }
    /**
     * @brief 保存旋转/尺度组合列表。
     * @param infos 要保存的组合列表。
     * @param path 输出 YAML 文件路径。
     */
    static void save_infos(std::vector<shapeInfo_producer::Info>& infos, std::string path = "infos.yaml"){
        cv::FileStorage fs(path, cv::FileStorage::WRITE);

        fs << "infos"
           << "[";
        for (int i = 0; i < infos.size(); i++)
        {
            fs << "{";
            fs << "angle" << infos[i].angle;
            fs << "scale" << infos[i].scale;
            fs << "}";
        }
        fs << "]";
    }
    /**
     * @brief 读取旋转/尺度组合列表。
     * @param path 输入 YAML 文件路径。
     * @return 组合列表。
     */
    static std::vector<Info> load_infos(std::string path = "info.yaml"){
        cv::FileStorage fs(path, cv::FileStorage::READ);

        std::vector<Info> infos;

        cv::FileNode infos_fn = fs["infos"];
        cv::FileNodeIterator it = infos_fn.begin(), it_end = infos_fn.end();
        for (int i = 0; it != it_end; ++it, i++)
        {
            infos.emplace_back(float((*it)["angle"]), float((*it)["scale"]));
        }
        return infos;
    }

    /// @brief 根据 angle_range/scale_range 生成全部旋转/尺度组合。
    void produce_infos(){
        infos.clear();

        assert(angle_range.size() <= 2);
        assert(scale_range.size() <= 2);
        assert(angle_step > eps*10);
        assert(scale_step > eps*10);

        // make sure range not empty
        if(angle_range.size() == 0){
            angle_range.push_back(0);
        }
        if(scale_range.size() == 0){
            scale_range.push_back(1);
        }

        if(angle_range.size() == 1 && scale_range.size() == 1){
            float angle = angle_range[0];
            float scale = scale_range[0];
            infos.emplace_back(angle, scale);

        }else if(angle_range.size() == 1 && scale_range.size() == 2){
            assert(scale_range[1] > scale_range[0]);
            float angle = angle_range[0];
            for(float scale = scale_range[0]; scale <= scale_range[1]+eps; scale += scale_step){
                infos.emplace_back(angle, scale);
            }
        }else if(angle_range.size() == 2 && scale_range.size() == 1){
            assert(angle_range[1] > angle_range[0]);
            float scale = scale_range[0];
            for(float angle = angle_range[0]; angle <= angle_range[1]+eps; angle += angle_step){
                infos.emplace_back(angle, scale);
            }
        }else if(angle_range.size() == 2 && scale_range.size() == 2){
            assert(scale_range[1] > scale_range[0]);
            assert(angle_range[1] > angle_range[0]);
            for(float scale = scale_range[0]; scale <= scale_range[1]+eps; scale += scale_step){
                for(float angle = angle_range[0]; angle <= angle_range[1]+eps; angle += angle_step){
                    infos.emplace_back(angle, scale);
                }
            }
        }
    }

    /**
     * @brief 生成指定旋转/尺度组合对应的模板图像。
     * @param info 旋转/尺度组合。
     * @return 变换后的模板图像。
     */
    cv::Mat src_of(const Info& info){
        return transform(src, info.angle, info.scale);
    }

    /**
     * @brief 生成指定旋转/尺度组合对应的 mask。
     * @param info 旋转/尺度组合。
     * @return 变换后的二值 mask。
     */
    cv::Mat mask_of(const Info& info){
        return (transform(mask, info.angle, info.scale) > 0);
    }
};

}

#endif
