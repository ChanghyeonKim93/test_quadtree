#include "quadtree/quadtree_pointer.h"

namespace PointerBased{
    Quadtree::Quadtree(
        float x_min, float x_max,
        float y_min, float y_max,
        std::shared_ptr<ObjectPool<QuadNode>> objpool_node, std::shared_ptr<ObjectPool<Elem>> objpool_elem,
        uint32_t max_depth, uint32_t max_elem_per_leaf,
        float approx_rate, uint8_t flag_adj)
    : max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max},
    n_node_activated_(1), max_elem_per_leaf_(max_elem_per_leaf)
    {
        std::cout << "sizeof(QuadNode): "<< sizeof(QuadNode) << "Bytes" << std::endl;

        // 
        objpool_node_ = objpool_node;
        objpool_elem_ = objpool_elem;

        // Initialize a root node.
        root_node_ = new QuadNode();
        root_node_->depth = 0;
        MAKE_LEAF_P(root_node_);

        root_node_->id_elemptrs = vector_elemptrs_.size();
        vector_elemptrs_.push_back(ElemPtrs());

        // Root size.
        QuadUint br_x = 1 << 15;
        QuadUint br_y = 1 << 15;
        
        root_node_->rect.tl = Pos2d<QuadUint>(0, 0);
        root_node_->rect.br = Pos2d<QuadUint>(br_x, br_y);

        // Calculate normalizers 
        x_normalizer_ = (float)root_node_->rect.br.x/(x_range_[1] - x_range_[0]);
        y_normalizer_ = (float)root_node_->rect.br.y/(y_range_[1] - y_range_[0]);

        if(x_normalizer_ > y_normalizer_) y_normalizer_ = x_normalizer_;
        else x_normalizer_ = y_normalizer_;

        // Parameter setting
        params_.approx_rate = approx_rate;
        if(params_.approx_rate > 1.0f) params_.approx_rate = 1.0f;
        if(params_.approx_rate < 0.5f) params_.approx_rate = 0.5f;
        params_.approx_rate = params_.approx_rate*params_.approx_rate;

        params_.flag_adj_search_only = flag_adj;
    };

    Quadtree::~Quadtree(){
        delete root_node_;
        for(auto const& it : nodes_){
            it->reset();
            objpool_node_->returnObject(it);
        }
        for(auto const& it : elements_){
            objpool_elem_->returnObject(it);
        }
        std::cout << "Quadtree is deleted.\n";
    }


    void Quadtree::resetNNParameters(){
        // Initialize values
        simple_stack_.clear();
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_  = std::numeric_limits<float>::max();
    };

    inline void Quadtree::resetQueryData(){
        query_data_.id_data_matched = 0;
        query_data_.ptr_node_cached  = root_node_;
        query_data_.ptr_node_matched = root_node_;
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_  = std::numeric_limits<float>::max();
        query_data_.x     = -1.0f;
        query_data_.y     = -1.0f;
        query_data_.x_nom = -1.0f;
        query_data_.y_nom = -1.0f;   
    };



    void Quadtree::insert(float x, float y, int id_data){
        if( x > x_range_[0] && x < x_range_[1]
        &&  y > y_range_[0] && y < y_range_[1] )
        {
            // normalize input point coordinates
            float x_nom = x*x_normalizer_; float y_nom = y*y_normalizer_;

            // Add the input data as an Element object
            ElemPtr elem_new = objpool_elem_->getObject();
            elem_new->resetAndSetData(x_nom, y_nom, id_data);

            elements_.push_back(elem_new);

            // Initialize the 'insert_data_' 
            // This is to avoid recursive copies of recursive call of function inputs) 
            insert_data_.resetAndSetData(x_nom, y_nom, elem_new);

            // Insert query_data_ into the quadtree.
            // this->insertPrivate(1, 0); // Recursion-based approach (DFS)
            this->insertPrivateStack(); // Stack-based approach (DFS), faster than Recursion-based one 1.2 times.
        }
        else throw std::runtime_error("Input x,y is out of the quadtree range.");
    };

    void Quadtree::insertPrivateStack()
    {
        float& x_nom = insert_data_.x_nom;
        float& y_nom = insert_data_.y_nom;

        QuadNodePtr ptr_node = root_node_;
        ptr_node->depth = 0;

        while(true){
            
            int depth = ptr_node->depth;

            if(IS_BRANCH_P(ptr_node)){
                // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
                Flag flag_sn, flag_ew;
                FIND_QUADRANT(x_nom, y_nom, ptr_node->rect, flag_sn, flag_ew);
                uint8_t idx_child = GET_CHILD_INDEX(flag_sn, flag_ew);

                ptr_node = ptr_node->first_child + idx_child;
                ptr_node->depth = depth + 1;
            }
            else if(IS_LEAF_P(ptr_node)){
                addDataToNode(ptr_node, insert_data_.elem);

                if(depth < max_depth_){
                    // make all child to leaf (with no element)
                    int n_elem = getNumElemOfNode(ptr_node);
                    if(n_elem > max_elem_per_leaf_){
                        makeChildrenLeaves(ptr_node);
                        n_node_activated_ += 4;

                        // Do divide
                        for(ElemPtr const& elem_tmp : vector_elemptrs_[ptr_node->id_elemptrs].elems){
                            Flag flag_sn, flag_ew;
                            FIND_QUADRANT(elem_tmp->x_nom, elem_tmp->y_nom, ptr_node->rect, flag_sn, flag_ew);
                            uint8_t idx_child = GET_CHILD_INDEX(flag_sn, flag_ew);

                            addDataToNode(ptr_node->first_child + idx_child, elem_tmp);
                        }
                        // make this node as a branch
                        makeBranch(ptr_node);
                    }
                }
                break;
            }
        }
    };


    void Quadtree::NNSearch(float x, float y,
        ID& id_data_matched, QuadNodePtr& ptr_node_matched)
    {
        query_data_.x = x; 
        query_data_.y = y;
        query_data_.x_nom = x*x_normalizer_;
        query_data_.y_nom = y*y_normalizer_;
        // query_data_.ptr_node_cached = root_node_; // non-cached case now.

        nearestNeighborSearchPrivate();

        id_data_matched  = query_data_.id_data_matched;
        ptr_node_matched = query_data_.ptr_node_matched;
    };

    void Quadtree::NNSearchDebug(float x, float y, 
            ID& id_data_matched, QuadNodePtr& ptr_node_matched, uint32_t& n_access)
    {
        NNSearch(x,y, id_data_matched, ptr_node_matched);
        n_access = simple_stack_.getTotalAccess();
    };

    void Quadtree::cachedNNSearchDebug(float x, float y, QuadNodePtr ptr_node_cached, 
        ID& id_data_matched, QuadNodePtr& ptr_node_matched, uint32_t& n_access)
    {
        cachedNNSearch(x,y, ptr_node_cached, id_data_matched, ptr_node_matched);
        n_access = simple_stack_.getTotalAccess();
    }
     
    inline void Quadtree::addDataToNode(QuadNodePtr ptr_node, ElemPtr elem){
        vector_elemptrs_[ptr_node->id_elemptrs].elems.push_back(elem);
    };

    bool Quadtree::findNearestElem(float x, float y, QuadNodePtr ptr_node){
        bool findNewNearest = false;
        for(ElemPtr const& elem : vector_elemptrs_[ptr_node->id_elemptrs].elems){
            float dist_temp = DIST_EUCLIDEAN(x,y, elem->x_nom, elem->y_nom);
            if(dist_temp < query_data_.min_dist2_){
                this->query_data_.id_data_matched = elem->id_data;
                this->query_data_.min_dist2_      = dist_temp * params_.approx_rate;
                this->query_data_.min_dist_       = sqrt(query_data_.min_dist2_);
                findNewNearest  = true;
            }
        }
        return findNewNearest;
    };

    void Quadtree::cachedNNSearch(float x, float y, QuadNodePtr ptr_node_cached, 
            ID& id_data_matched, QuadNodePtr& ptr_node_matched)
    {
        query_data_.x = x;
        query_data_.y = y;
        query_data_.x_nom = x*x_normalizer_;
        query_data_.y_nom = y*y_normalizer_;
        query_data_.ptr_node_cached = ptr_node_cached; 

        cachedNearestNeighborSearchPrivate();

        id_data_matched  = query_data_.id_data_matched;
        ptr_node_matched = query_data_.ptr_node_matched;
    };

    inline int Quadtree::getNumElemOfNode(QuadNodePtr ptr_node){
        return vector_elemptrs_[ptr_node->id_elemptrs].elems.size();
    };

    inline void Quadtree::makeChildrenLeaves(QuadNodePtr ptr_parent){
        QuadRect_u& rect = ptr_parent->rect;
        QuadUint cent_x = (rect.tl.x + rect.br.x) >> 1;
        QuadUint cent_y = (rect.tl.y + rect.br.y) >> 1;
        
        // Get four objects consecutive memories
        ptr_parent->first_child = objpool_node_->getObjectQuadruple();
        nodes_.push_back(ptr_parent->first_child   );
        nodes_.push_back(ptr_parent->first_child + 1);
        nodes_.push_back(ptr_parent->first_child + 2);
        nodes_.push_back(ptr_parent->first_child + 3);

        QuadNodePtr ptr_child = ptr_parent->first_child;

        ptr_child->reset();// (0,0) (top left)
        MAKE_LEAF_P(ptr_child);
        ptr_child->rect.tl.x = rect.tl.x; ptr_child->rect.tl.y = rect.tl.y; 
        ptr_child->rect.br.x = cent_x;    ptr_child->rect.br.y = cent_y;
        ptr_child->parent = ptr_parent;
        ptr_child->depth = ptr_parent->depth + 1;
        ptr_child->id_elemptrs = vector_elemptrs_.size();
        vector_elemptrs_.push_back(ElemPtrs());

        (++ptr_child)->reset();// (0,1) (top right)
        MAKE_LEAF_P(ptr_child); 
        ptr_child->rect.tl.x = cent_x;    ptr_child->rect.tl.y = rect.tl.y; 
        ptr_child->rect.br.x = rect.br.x; ptr_child->rect.br.y = cent_y;
        ptr_child->parent = ptr_parent;
        ptr_child->depth = ptr_parent->depth + 1;
        ptr_child->id_elemptrs = vector_elemptrs_.size();
        vector_elemptrs_.push_back(ElemPtrs());


        (++ptr_child)->reset(); // (1,0) (bot left)
        MAKE_LEAF_P(ptr_child);
        ptr_child->rect.tl.x = rect.tl.x; ptr_child->rect.tl.y = cent_y; 
        ptr_child->rect.br.x = cent_x;    ptr_child->rect.br.y = rect.br.y;
        ptr_child->parent = ptr_parent;
        ptr_child->depth = ptr_parent->depth + 1;
        ptr_child->id_elemptrs = vector_elemptrs_.size();
        vector_elemptrs_.push_back(ElemPtrs());


        (++ptr_child)->reset(); // (1,1) (bot right)
        MAKE_LEAF_P(ptr_child);
        ptr_child->rect.tl.x = cent_x;    ptr_child->rect.tl.y = cent_y; 
        ptr_child->rect.br.x = rect.br.x; ptr_child->rect.br.y = rect.br.y;
        ptr_child->parent = ptr_parent;
        ptr_child->depth = ptr_parent->depth + 1;
        ptr_child->id_elemptrs = vector_elemptrs_.size();
        vector_elemptrs_.push_back(ElemPtrs());
    };

    void Quadtree::nearestNeighborSearchPrivate(){
        float x_nom = query_data_.x_nom;
        float y_nom = query_data_.y_nom;
        QuadNodePtr& ptr_node_matched = query_data_.ptr_node_matched;

        // Initialize NN parameters
        resetNNParameters();

        // Start from the root node.
        // Do Breadth First Search (BFS) 
        simple_stack_.push(root_node_); 
        while(!simple_stack_.empty()){
            QuadNodePtr ptr_node = simple_stack_.topAndPop();
            // Current depth <= max_depth_ ?? 

            // If leaf node, find nearest point and 'BWBTest()'
            // if(nd.isLeaf()){
            if(IS_LEAF_P(ptr_node)){
                simple_stack_.addTotalAccess();
                if(findNearestElem(x_nom, y_nom, ptr_node)) {
                    ptr_node_matched = ptr_node;
                }
                
                if( BWBTest(x_nom, y_nom, ptr_node->rect, query_data_.min_dist_) ) break; // the nearest point is inside the node.
            }
            else { // this is not a leaf node.
                simple_stack_.addTotalAccess();

                // Go to child. Find most probable child first.
                QuadNodePtr ptr_child = ptr_node->first_child;
                if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                    simple_stack_.push(ptr_child);
                }
                ++ptr_child;
                if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                    simple_stack_.push(ptr_child);
                } 
                ++ptr_child;
                if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                    simple_stack_.push(ptr_child);
                }
                ++ptr_child;
                if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                    simple_stack_.push(ptr_child);
                }
            }
        }
    };

    void Quadtree::cachedNearestNeighborSearchPrivate(){
        // In this function, search the nearest element from the scratch (from the root node)
        float x_nom = query_data_.x_nom;
        float y_nom = query_data_.y_nom;

        // Initialize NN parameters
        resetNNParameters();

        // Start from the cached node. 
        // The cached node should be a leaf node.
        // If the cached node is invalid or the root node, normal NN search is executed.
        if(query_data_.ptr_node_cached == root_node_){ 
            nearestNeighborSearchPrivate();
            return;
        }
        else // cached node is neither a root node nor a invalid node.
        {  
            query_data_.ptr_node_matched = query_data_.ptr_node_cached;
            simple_stack_.addTotalAccess();

            // Find nearest point in the cached node.
            findNearestElem(x_nom, y_nom, query_data_.ptr_node_matched);
            if( BWBTest(x_nom, y_nom, query_data_.ptr_node_matched->rect, query_data_.min_dist_) ) {
                // the nearest point is inside the cached node.
                return;
            } 

            // Go up to the until satisfying 'BWBTest == true'
            QuadNodePtr ptr_node = query_data_.ptr_node_cached->parent;
            while(true){
                simple_stack_.addTotalAccess();

                if(ptr_node == root_node_){
                    break; // reach the root node.
                } 
                // If 'BWBTest' is passed on this node, the nearest one should be within
                // the chilren of this node. 
                if(BWBTest(x_nom, y_nom, ptr_node->rect, query_data_.min_dist_)) break; 

                ptr_node = ptr_node->parent;
            }
                
            // Start from the root node.
            // Do Breadth First Search (BFS) 
            simple_stack_.push(ptr_node); 
            while(!simple_stack_.empty()){
                QuadNodePtr ptr_node = simple_stack_.topAndPop();
                // Current depth <= max_depth_ ?? 

                // If leaf node, find nearest point and 'BWBTest()'
                // if(nd.isLeaf()){
                if(IS_LEAF_P(ptr_node)){
                    simple_stack_.addTotalAccess();
                    if(findNearestElem(x_nom, y_nom, ptr_node)) {
                        query_data_.ptr_node_matched = ptr_node;
                    }
                    
                    if( BWBTest(x_nom, y_nom, ptr_node->rect, query_data_.min_dist_) ) break; // the nearest point is inside the node.
                }
                else { // this is not a leaf node.
                    simple_stack_.addTotalAccess();

                    // Go to child. Find most probable child first.
                    QuadNodePtr ptr_child = ptr_node->first_child;
                    if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                        simple_stack_.push(ptr_child);
                    }
                    ++ptr_child;
                    if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                        simple_stack_.push(ptr_child);
                    } 
                    ++ptr_child;
                    if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                        simple_stack_.push(ptr_child);
                    }
                    ++ptr_child;
                    if(IS_ACTIVATED_P(ptr_child) && BOBTest(x_nom, y_nom, ptr_child->rect, query_data_.min_dist_)) {
                        simple_stack_.push(ptr_child);
                    }
                }
            }
        }
    };

    inline bool Quadtree::BWBTest(float x, float y, const QuadRect_u& rect, float radius){
        // Ball Within Bound (Circle bounded by rect check)
        float rect_size = (float)(rect.br.x-rect.tl.x);
        return (rect_size >= 2*radius) && INBOUND_RECT_PTS(x, y, rect.tl.x+radius,rect.tl.y+radius, rect.br.x-radius, rect.br.y-radius);
    };


    inline bool Quadtree::BOBTest(float x, float y, const QuadRect_u& rect, float R){
        // Ball Overlap Bound (Circle-rect collision check)
        float rect_center_x   = (float)(rect.br.x+rect.tl.x)*0.5f;
        float rect_center_y   = (float)(rect.br.y+rect.tl.y)*0.5f;
        float rect_halfsize_x = (float)(rect.br.x-rect.tl.x)*0.5f;
        float rect_halfsize_y = (float)(rect.br.y-rect.tl.y)*0.5f;

        float dx = fabs(x - rect_center_x);
        float dy = fabs(y - rect_center_y);
        
        if(dx > (rect_halfsize_x + R) ) return false;
        if(dy > (rect_halfsize_y + R) ) return false;

        if(dx <= rect_halfsize_x) return true;
        if(dy <= rect_halfsize_y) return true;

        float corner_dist_sq = (dx-rect_halfsize_x)*(dx-rect_halfsize_x)
                            + (dy-rect_halfsize_y)*(dy-rect_halfsize_y);
        
        return (corner_dist_sq <= (R*R));
    };

    inline void Quadtree::makeBranch(QuadNodePtr ptr_node){
        MAKE_BRANCH_P(ptr_node);
        vector_elemptrs_[ptr_node->id_elemptrs].reset();
    };

    uint32_t Quadtree::getNumNodesActivated(){
        return n_node_activated_;
    };

};