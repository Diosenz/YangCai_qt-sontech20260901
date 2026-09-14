// (c) Matthew B. Kennel, Institute for Nonlinear Science, UCSD (2004)
//
// Licensed under the Academic Free License version 1.1 found in file LICENSE
// with additional provisions in that same file.
//
// Implement a kd tree for fast searching of points in a fixed data base
// in k-dimensional Euclidean space.

/**
 * @file kdtree.h
 * @brief libicp 使用的 kd-tree 最近邻搜索结构。
 * @details 该文件保留 Matthew B. Kennel 的 AFL 1.1 授权说明。当前工程只补充
 *          Doxygen 注释和 include 顺序，不改变 kd-tree 搜索行为。
 */

#ifndef I2R_RLC_VISION_ICP_LIBICP_KDTREE_H
#define I2R_RLC_VISION_ICP_LIBICP_KDTREE_H

#include <algorithm>
#include <vector>

#include <boost/array.hpp>
#include <boost/multi_array.hpp>

namespace kdtree {
  
  /// @brief kd-tree 使用的二维点云存储，第一维为点索引，第二维为坐标维度。
  typedef boost::multi_array<float, 2>           KDTreeArray;
  /// @brief 只读点云视图。
  typedef boost::const_multi_array_ref<float, 2> KDTreeROArray;
  
  /// @brief 单个坐标轴上的包围区间。
  typedef struct {
    float lower, upper;
  } interval;
  
  // let the compiler know that this is a names of classes.
  class KDTreeNode;
  class SearchRecord;  
  
  /// @brief 最近邻查询结果，`dis` 为平方距离，`idx` 为点索引。
  struct KDTreeResult {
  public:
    float dis;  // square distance
    int idx;    // neighbor index
  };
  
  /**
   * @brief 最近邻结果容器。
   *
   * 继承 vector 是旧实现接口的一部分；可选地维护为堆，用于保留最大优先级元素。
   */
  class KDTreeResultVector : public std::vector<KDTreeResult> {
    // inherit a std::vector<KDTreeResult>
    // but, optionally maintain it in heap form as a priority
    // queue.
  public:
    
    //
    // add one new element to the list of results, and
    // keep it in heap order.  To keep it in ordinary, as inserted,
    // order, then simply use push_back() as inherited
    // via std::vector<>
    
    void push_element_and_heapify(KDTreeResult&);
    float replace_maxpri_elt_return_new_maxpri(KDTreeResult&);
    
    float max_value();
    // return the distance which has the maximum value of all on list,
    // assuming that ALL insertions were made by
    // push_element_and_heapify()
  };
  
  /**
   * @brief 固定点云上的 kd-tree。
   *
   * 对输入点云只持有引用；调用方必须保证底层 `KDTreeArray` 生命周期长于树对象。
   */
  class KDTree {
  public:

    /// @brief 原始点云引用，树对象不拥有该存储。
    const KDTreeArray& the_data;
    // "the_data" is a reference to the underlying multi_array of the
    // data to be included in the tree.
    //
    // NOTE: this structure does *NOT* own the storage underlying this.
    // Hence, it would be a very bad idea to change the underlying data
    // during use of the search facilities of this tree.
    // Also, the user must deallocate the memory underlying it.
    
    
    const int N;          ///< 数据点数量。
    /// @brief 搜索使用的坐标维度。
    int m_dim;
    bool sort_results;    ///< 是否对查询结果排序。
    const bool rearrange; ///< 是否在内部重排点云。
 
  public:

    // constructor, has optional 'dim_in' feature, to use only
    // first 'dim_in' components for definition of nearest neighbors.    
    /**
     * @brief 构造 kd-tree。
     * @param data_in 输入点云，第一维为点索引，第二维为坐标。
     * @param rearrange_in 是否在内部重排点云以加速搜索。
     * @param dim_in 搜索维度；-1 表示使用输入点云全部维度。
     */
    KDTree(KDTreeArray& data_in, bool rearrange_in = true, int dim_in=-1);
    
    // destructor
    /// @brief 析构并释放树节点。
    ~KDTree();
    
    
  public:
    
    /// @brief 暴力搜索 n 个最近邻，主要用于调试。
    void n_nearest_brute_force(std::vector<float>& qv, int nn, KDTreeResultVector& result);
    // search for n nearest to a given query vector 'qv' usin
    // exhaustive slow search.  For debugging, usually.
    
    /// @brief 搜索查询向量的 n 个最近邻。
    void n_nearest(std::vector<float>& qv, int nn, KDTreeResultVector& result);
    // search for n nearest to a given query vector 'qv'.
    
    /// @brief 围绕已有点搜索最近邻，并排除相关时间窗口内的点。
    void n_nearest_around_point(int idxin, int correltime, int nn,
            KDTreeResultVector& result);
    // search for 'nn' nearest to point [idxin] of the input data, excluding
    // neighbors within correltime
    
    /// @brief 搜索平方距离不大于 `r2` 的所有邻居。
    void r_nearest(std::vector<float>& qv, float r2, KDTreeResultVector& result);
    // search for all neighbors in ball of size (square Euclidean distance)
    // r2. Return number of neighbors in 'result.size()',
    
    /// @brief 围绕已有点执行半径搜索，并排除相关时间窗口内的点。
    void r_nearest_around_point(int idxin, int correltime, float r2,
            KDTreeResultVector& result);
    // like 'r_nearest', but around existing point, with decorrelation
    // interval.
    
    /// @brief 统计查询向量半径内邻居数量。
    int r_count(std::vector<float>& qv, float r2);
    // count number of neighbors within square distance r2.
    
    /// @brief 统计已有点半径内邻居数量，并排除相关时间窗口内的点。
    int r_count_around_point(int idxin, int correltime, float r2);
    // like r_count, c
    
    friend class KDTreeNode;
    friend class SearchRecord;
    
  private:
    
    KDTreeNode* root; // the root pointer
    
    const KDTreeArray* data;
    // pointing either to the_data or an internal
    // rearranged data as necessary
    
    std::vector<int> ind;
    // the index for the tree leaves.  Data in a leaf with bounds [l,u] are
    // in  'the_data[ind[l],*] to the_data[ind[u],*]
    
    KDTreeArray rearranged_data;
    // if rearrange is true then this is the rearranged data storage.

    //static const int bucketsize = 12;  // global constant.
	enum{bucketsize = 12};

  private:
    void set_data(KDTreeArray& din);
    void build_tree(); // builds the tree.  Used upon construction.
    KDTreeNode* build_tree_for_range(int l, int u, KDTreeNode* parent);
    void select_on_coordinate(int c, int k, int l, int u);
    int select_on_coordinate_value(int c, float alpha, int l, int u);
    void spread_in_coordinate(int c, int l, int u, interval& interv);
  };
  
  /// @brief kd-tree 节点，保存切分维度、切分值和包围盒。
  class KDTreeNode {
  public:
    KDTreeNode(int m_dim);
    ~KDTreeNode();
    
  private:
    // visible to self and KDTree.
    friend class KDTree;  // allow kdtree to access private data
    
    int cut_dim;                                 // dimension to cut;
    float cut_val, cut_val_left, cut_val_right;  //cut value
    int l, u;  // extents in index array for searching
    
    std::vector<interval> box; // [min,max] of the box enclosing all points
    
    KDTreeNode *left, *right;  // pointers to left and right nodes.
    
    void search(SearchRecord& sr);
    // recursive innermost core routine for searching..
    
    bool box_in_search_range(SearchRecord& sr);
    // return true if the bounding box for this node is within the
    // search range given by the searchvector and maximum ballsize in 'sr'.
    
    void check_query_in_bound(SearchRecord& sr); // debugging only
    
    // for processing final buckets.
    void process_terminal_node(SearchRecord& sr);
    void process_terminal_node_fixedball(SearchRecord& sr); 
  };
  
} // namespace kdtree

#endif


