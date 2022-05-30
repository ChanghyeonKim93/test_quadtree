#include "quadtree_fast2.h"

Quadtree::Quadtree(
    float x_min, float x_max,
    float y_min, float y_max,
    uint32_t max_depth, uint32_t max_elem_per_leaf)
: max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max}
{
    // Make nodes
    std::cout << "max_depth:" << max_depth_ <<std::endl;
    n_nodes_ = 1*(std::pow(4,max_depth_+1)-1)/(4-1);
    nodes.resize(n_nodes_+1);
    // nodes[1].makeLeaf();
    MAKE_LEAF(nodes[1]); // root node
    std::cout << "# of nodes: " << nodes.size() << std::endl;

    node_elements.resize(nodes.size());
    std::cout <<"max depth nodes range: " << 1+(std::pow(4,max_depth_)-1)/3 <<"~" <<(std::pow(4,max_depth_+1)-1)/3<<std::endl;

    // Max depth and normalized tree size 
    uint32_t br_x = 1 << 16;
    uint32_t br_y = 1 << 16;
    
    nodes[1].rect.tl = Pos2d<uint32_t>(0, 0);
    nodes[1].rect.br = Pos2d<uint32_t>(br_x, br_y);
    std::cout << "grid size: " << nodes[1].rect <<std::endl;

    // Normalizer
    x_normalizer_ = (float)nodes[1].rect.br.x/(x_range_[1] - x_range_[0]);
    y_normalizer_ = (float)nodes[1].rect.br.y/(y_range_[1] - y_range_[0]);
    x_dist_weight_ = 1.0f/x_normalizer_;
    y_dist_weight_ = 1.0f/y_normalizer_;

    std::cout << "xy normalizer: " << x_normalizer_ << "," << y_normalizer_ << std::endl;
    std::cout << "xy minimum grid size [px]: " 
        << 1./x_normalizer_ << "," << 1./y_normalizer_ << std::endl;
        
    max_elem_per_leaf_ = max_elem_per_leaf;

    std::cout <<"sizeof(QuadNode):" << sizeof(Quadtree::QuadNode) << std::endl;
};

Quadtree::~Quadtree() { 
    // there is no dynamic memory allocation.
    std::cout << "Quadtree is deleted.\n";
};

void Quadtree::insert(float x, float y, int id_data){
    if( x > x_range_[0] && x < x_range_[1]
    &&  y > y_range_[0] && y < y_range_[1] )
    {
        float x_nom = x*x_normalizer_; float y_nom = y*y_normalizer_;
#ifdef VERBOSE_
        std::cout << "\n======== insert [" << x_nom << "," << y_nom << "] ========\n"; 
#endif
  
        all_elems_.push_back(QuadElement(x_nom,y_nom, (ID)id_data));
        insert_data_.setData(x_nom, y_nom, all_elems_.size()-1);

        this->insertPrivate(1, 0);
    }
    else throw std::runtime_error("Input x,y is out of the quadtree range.");
};

void Quadtree::insertPrivate(ID id_node, uint8_t depth)
{
    QuadNode&              nd = nodes[id_node]; // current node.
    QuadNodeElements& ndelems = node_elements[id_node];
    nd.depth = depth;
    
    float& x_nom = insert_data_.x_nom;
    float& y_nom = insert_data_.y_nom;

    if(IS_BRANCH(nd)) {
        // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
        Flag flag_sn, flag_ew;
        FIND_QUADRANT(x_nom, y_nom, nd.rect,      flag_sn, flag_ew);
        ID id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
        
        insertPrivate(id_child, depth + 1); // Go to the child
    }
    else if(IS_LEAF(nd)) { // This is a leaf node.
        addDataToNode(id_node, insert_data_.id_elem);

        if(depth < max_depth_){ // nonmax depth.
            int n_elem = getNumElemOfNode(id_node);
            if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                // Make all child to leaf (with no element)
                ID id_child = GET_FIRST_CHILD_ID(id_node);
                makeChildrenLeaves(id_child, nd.rect);

                // Do divide.
                for(const ID& id_elem_tmp : ndelems.elem_ids) {
                    Flag flag_sn, flag_ew;
                    QuadElement& elem_tmp = all_elems_[id_elem_tmp];
                    FIND_QUADRANT(elem_tmp.x_nom, elem_tmp.y_nom, nd.rect, flag_sn, flag_ew);
                    ID id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
                    addDataToNode(id_child, id_elem_tmp);
                }

                //make this node as a branch
                makeBranch(id_node);
            }
        }
    }
};

inline void Quadtree::getQuadrant(float x, float y, const QuadRect_u32& qrect, Flag& flag_sn, Flag& flag_ew){
    uint32_t cent_x = ((qrect.tl.x + qrect.br.x) >> 1);
    uint32_t cent_y = ((qrect.tl.y + qrect.br.y) >> 1);
    flag_ew = x > cent_x; // west : 0, east : 1
    flag_sn = y > cent_y; // north: 0, south: 1
};

inline void Quadtree::addDataToNode(ID id_node, ID id_elem){
    node_elements[id_node].elem_ids.push_back(id_elem);
};

inline int Quadtree::getNumElemOfNode(ID id_node){
    return node_elements[id_node].elem_ids.size();
};


inline void Quadtree::makeChildrenLeaves(ID id_child, const QuadRect_u32& rect){
    uint32_t cent_x = (rect.tl.x + rect.br.x) >> 1;
    uint32_t cent_y = (rect.tl.y + rect.br.y) >> 1;

    MAKE_LEAF(nodes[id_child]); // (0,0) (top left)
    nodes[id_child].rect.tl.x = rect.tl.x; nodes[id_child].rect.tl.y = rect.tl.y; 
    nodes[id_child].rect.br.x = cent_x;    nodes[id_child].rect.br.y = cent_y;

    MAKE_LEAF(nodes[++id_child]); // (0,1) (top right)
    nodes[id_child].rect.tl.x = cent_x;    nodes[id_child].rect.tl.y = rect.tl.y; 
    nodes[id_child].rect.br.x = rect.br.x; nodes[id_child].rect.br.y = cent_y;

    MAKE_LEAF(nodes[++id_child]); // (1,0) (bot left)
    nodes[id_child].rect.tl.x = rect.tl.x; nodes[id_child].rect.tl.y = cent_y; 
    nodes[id_child].rect.br.x = cent_x;    nodes[id_child].rect.br.y = rect.br.y;

    MAKE_LEAF(nodes[++id_child]); // (1,1) (bot right)
    nodes[id_child].rect.tl.x = cent_x;    nodes[id_child].rect.tl.y = cent_y; 
    nodes[id_child].rect.br.x = rect.br.x; nodes[id_child].rect.br.y = rect.br.y;
};

inline void Quadtree::makeBranch(ID id_node){
    MAKE_BRANCH(nodes[id_node]);
    node_elements[id_node].reset();
};

inline bool Quadtree::BWBTest(float x, float y, const QuadRect_u32& rect, float radius){
    // Ball Within Bound (Circle bounded by rect check)
    float rect_size = (float)(rect.br.x-rect.tl.x);
    return (rect_size >= 2*radius) && INBOUND_RECT_PTS(x, y, rect.tl.x+radius,rect.tl.y+radius, rect.br.x-radius, rect.br.y-radius);
};

inline bool Quadtree::BOBTest(float x, float y, const QuadRect_u32& rect, float R){
    // Ball Overlap Bound (Circle-rect collision check)
    float rect_center_x   = (float)(rect.br.x+rect.tl.x)*0.5f;
    float rect_center_y   = (float)(rect.br.y+rect.tl.y)*0.5f;
    float rect_halfsize_x = (float)(rect.br.x-rect.tl.x)*0.5f;
    float rect_halfsize_y = (float)(rect.br.y-rect.tl.y)*0.5f;

    float dx = fabs(x - rect_center_x);
    float dy = fabs(y - rect_center_y);
    
    if(dx > (rect_halfsize_x + R)) return false;
    if(dy > (rect_halfsize_y + R)) return false;

    if(dx <= (rect_halfsize_x)) return true;
    if(dy <= (rect_halfsize_y)) return true;

    float corner_dist_sq = (dx-rect_halfsize_x)*(dx-rect_halfsize_x)
        + (dy-rect_halfsize_y)*(dy-rect_halfsize_y);
    
    return (corner_dist_sq <= (R*R));
};

void Quadtree::resetNNParameters(){
    // Initialize values
    simple_stack_.clear();
    query_data_.min_dist2_ = std::numeric_limits<float>::max();
    query_data_.min_dist_  = std::numeric_limits<float>::max();
    min_dist2_ = std::numeric_limits<float>::max();
};

void Quadtree::nearestNeighborSearchPrivate(){
    // In this function, search the nearest element from the scratch (from the root node)
    float x_nom = query_data_.x_nom;
    float y_nom = query_data_.y_nom;
    ID& id_data_matched = query_data_.id_data_matched;
    ID& id_elem_matched = query_data_.id_elem_matched;
    ID& id_node_matched = query_data_.id_node_matched;

    // Initialize NN parameters
    resetNNParameters();

    // Start from the root node.
    // Do Breadth First Search (BFS) 
    simple_stack_.push(1); 
    while(!simple_stack_.empty()){
        ID id_node   = simple_stack_.topAndPop();
        QuadNode& nd = nodes[id_node];
        // Current depth <= max_depth_ ?? 

        // If leaf node, find nearest point and 'BWBTest()'
        // if(nd.isLeaf()){
        if(IS_LEAF(nd)){
            simple_stack_.addTotalAccess();
            QuadNodeElements& nd_elems = node_elements[id_node];
            if(findNearestElem(x_nom, y_nom, nd_elems)) {
                id_node_matched = id_node;
            }

            if( BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_) ) break; // the nearest point is inside the node.
        }
        else { // this is not a leaf node.
            simple_stack_.addTotalAccess();

            // Go to child. Find most probable child first.
            ID id_child = GET_FIRST_CHILD_ID(id_node);
            if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                simple_stack_.push(id_child);
            }
            ++id_child;
            if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                simple_stack_.push(id_child);
            }
            ++id_child;
            if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                simple_stack_.push(id_child);
            }
            ++id_child;
            if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                simple_stack_.push(id_child);
            }
        }
    }
#ifdef VERBOSE_
    std::cout <<"\n --- statistics - # access nodes: " << simple_stack_.getTotalAccess() << std::endl;
#endif
};



bool Quadtree::findNearestElem(float x, float y, const QuadNodeElements& elems_thisnode){
    bool findNewNearest = false;
    for(const ID& id_elem : elems_thisnode.elem_ids){
        QuadElement& elem = all_elems_[id_elem];
        float dist_temp = DIST_EUCLIDEAN(x,y, elem.x_nom,elem.y_nom);
        if(dist_temp < min_dist2_){
            min_dist2_      = dist_temp;
            this->query_data_.id_data_matched = elem.id_data;
            this->query_data_.id_elem_matched = id_elem;
            this->query_data_.min_dist2_      = dist_temp;
            this->query_data_.min_dist_       = sqrt(dist_temp);
            findNewNearest  = true;
        }
    }
    return findNewNearest;
};

void Quadtree::cachedNearestNeighborSearchPrivate(){
    // In this function, search the nearest element from the scratch (from the root node)
    float& x_nom = query_data_.x_nom;
    float& y_nom = query_data_.y_nom;
    ID& id_data_matched = query_data_.id_data_matched;
    ID& id_elem_matched = query_data_.id_elem_matched;

    // Initialize NN parameters
    resetNNParameters();

    // Start from the cached node. 
    // The cached node should be a leaf node.
    // If the cached node is invalid or the root node, normal NN search is executed.
    if(query_data_.id_node_cached <= 1){ 
        nearestNeighborSearchPrivate();
        return;
    }
    else // cached node is neither a root node nor a invalid node.
    {  
        query_data_.id_node_matched = query_data_.id_node_cached;
        simple_stack_.addTotalAccess();

        // Find nearest point in the cached node.
        findNearestElem(x_nom, y_nom, node_elements[query_data_.id_node_matched]);
        if( BWBTest(x_nom, y_nom, nodes[query_data_.id_node_matched].rect, query_data_.min_dist_) ) {
            // the nearest point is inside the cached node.
#ifdef VERBOSE_
            std::cout <<"\n --- statistics (cached) - # access nodes: " << simple_stack_.getTotalAccess() << std::endl;
#endif
            return;
        } 

        // Go up to the until satisfying 'BWBTest == true'
        ID id_node = GET_PARENT_ID(query_data_.id_node_cached);
        while(true){
            simple_stack_.addTotalAccess();

            if(id_node <= 1){
                id_node = 1;
                break; // reach the root node.
            } 
            // If 'BWBTest' is passed on this node, the nearest one should be within
            // the chilren of this node. 
            if(BWBTest(x_nom, y_nom, nodes[id_node].rect, query_data_.min_dist_)) break; 

            id_node = GET_PARENT_ID(id_node);
        }
            
        // Start from the root node.
        // Do Breadth First Search (BFS) 
        simple_stack_.push(id_node); 
        while(!simple_stack_.empty()){
            id_node = simple_stack_.topAndPop();
            QuadNode& nd = nodes[id_node];

            // If leaf node, find nearest point and 'BWBTest()'
            // if(nd.isLeaf()){
            if(IS_LEAF(nd)){
                simple_stack_.addTotalAccess();
                QuadNodeElements& nd_elems = node_elements[id_node];
                if(findNearestElem(x_nom, y_nom, nd_elems)) {
                    query_data_.id_node_matched = id_node;
                }

                if( BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_) ) break; // the nearest point is inside the node.
            }
            else{ // this is not a leaf node.
                simple_stack_.addTotalAccess();

                // Go to child. Find most probable child first.
                ID id_child = GET_FIRST_CHILD_ID(id_node);
                if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if(IS_ACTIVATED(nodes[id_child]) && BOBTest(x_nom, y_nom, nodes[id_child].rect, query_data_.min_dist_)) {
                    simple_stack_.push(id_child);
                }
            }
        }
#ifdef VERBOSE_
        std::cout <<"\n --- statistics (cached) - # access nodes: " << simple_stack_.getTotalAccess() << std::endl;
#endif
    }
};


// Find nearest element.
// return : uint32_t id_data_matched.
void Quadtree::NNSearch(float x, float y, 
    uint32_t& id_data_matched, uint32_t& id_node_matched)
{
    query_data_.x = x;
    query_data_.y = y;
    query_data_.x_nom = x*x_normalizer_;
    query_data_.y_nom = y*y_normalizer_;
    // query_data_.id_node_cached = 0; // non-cached case now.
    
    nearestNeighborSearchPrivate();
#ifdef VERBOSE_
    std::cout << "Query, matched: [" << query_data_.x_nom << "," << query_data_.y_nom << "] / [";
    std::cout << all_elems_[query_data_.id_elem_matched].x_nom
                            <<"," << all_elems_[query_data_.id_elem_matched].y_nom <<"]"
                            <<" / min dist: " << sqrt(query_data_.min_dist2_) << std::endl;
#endif
    id_data_matched = query_data_.id_data_matched;
    id_node_matched = query_data_.id_node_matched;
};

void Quadtree::NNSearchDebug(float x, float y, 
    uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access)
{
    NNSearch(x,y, id_data_matched, id_node_matched);
    n_access = simple_stack_.getTotalAccess();
};

void Quadtree::cachedNNSearch(float x, float y, int id_node_cached, 
    uint32_t& id_data_matched, uint32_t& id_node_matched)
{
    query_data_.x = x;
    query_data_.y = y;
    query_data_.x_nom = x*x_normalizer_;
    query_data_.y_nom = y*y_normalizer_;
    query_data_.id_node_cached = id_node_cached; 
    
    cachedNearestNeighborSearchPrivate();

#ifdef VERBOSE_
    std::cout << "Query, matched: [" << query_data_.x_nom << "," << query_data_.y_nom << "] / [";
    std::cout << all_elems_[query_data_.id_elem_matched].x_nom
                            <<"," << all_elems_[query_data_.id_elem_matched].y_nom <<"]"
                            <<" / min dist: " << sqrt(query_data_.min_dist2_) << std::endl;
#endif
    id_data_matched = query_data_.id_data_matched;
    id_node_matched = query_data_.id_node_matched;
};

void Quadtree::cachedNNSearchDebug(float x, float y, int id_node_cached, 
    uint32_t& id_data_matched, uint32_t& id_node_matched, uint32_t& n_access)
{
    cachedNNSearch(x,y, id_node_cached, id_data_matched, id_node_matched);
    n_access = simple_stack_.getTotalAccess();
};

inline void Quadtree::resetInsertData(){
    insert_data_.id_elem = 0; 
    insert_data_.x_nom   = -1.0f;
    insert_data_.y_nom   = -1.0f;
};

inline void Quadtree::resetQueryData(){
    query_data_.id_data_matched = 0;
    query_data_.id_elem_matched = 0;
    query_data_.id_node_cached  = 0;
    query_data_.id_node_matched = 0;
    query_data_.min_dist2_ = std::numeric_limits<float>::max();
    query_data_.min_dist_  = std::numeric_limits<float>::max();
    query_data_.x     = -1.0f;
    query_data_.y     = -1.0f;
    query_data_.x_nom = -1.0f;
    query_data_.y_nom = -1.0f;   
};