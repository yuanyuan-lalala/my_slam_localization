#include <glog/logging.h>
#include "../utils/point_types.h"
#include "queue"


struct kdTreeNode{

    int id_ = -1;
    int point_idx = 0;
    int axis_index_ = 0; 
    float split_thresh_ = 0.0;  
    kdTreeNode* left_ = nullptr;
    kdTreeNode* right_ = nullptr;
    bool isLeaf() const {left_ == nullptr && right_ == nullptr;}

};


struct nodeAndDistance{
    nodeAndDistance(kdTreeNode* node,float dis2):node_(node),distance2_(dis2){}

    kdTreeNode* node_ = nullptr;
    float distance2_ = 0;
    bool operator <(const nodeAndDistance& other) const {return distance2_ < other.distance2_;}

};

class kdTree{

    public:
    
    explicit kdTree() = default;
    
    ~kdTree(){ Clear(); }



    bool buildTree(const CloudPtr& cloud);
    
    
    bool getClosestPoint(const PointType& pt,std::vector<std::pair<size_t,size_t>>& matches,int k = 5);

    void setEnableANN(bool use_ann = true,float alpha = 0.1){
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    size_t size() const {return size_;}

    //清理数据
    void Clear();   

    void printAll();


    private:

    void insert(const IndexVec& points,kdTreeNode* node);

    bool findSplitAxisAndThresh(const IndexVec& point_idx,int& axis,float& th,IndexVec& left,IndexVec& right);

    void Reset();

    static inline float Dis2(const Vec3f& p1,const Vec3f& p2){return (p1 - p2).squaredNorm();}

    void knn(const Vec3f& pt,kdTreeNode* node,std::priority_queue<nodeAndDistance>& result) const;

    void computeDisForLeaf(const Vec3f& pt,kdTreeNode* node,std::priority_queue<nodeAndDistance>& result) const;

    bool needExpand(const Vec3f& pt,kdTreeNode* node,std::priority_queue<nodeAndDistance>& knn_result) const;



    int k_ = 5;
    std::shared_ptr<kdTreeNode> root_ = nullptr;
    std::vector<Vec3f> cloud_;
    std::unordered_map<int, kdTreeNode*> nodes_;

    size_t size_ = 0;
    int tree_node_id_ = 0;

    bool approximate_ = true;
    float alpha_ = 0.1;


};
