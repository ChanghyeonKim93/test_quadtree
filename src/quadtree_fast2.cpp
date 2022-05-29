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
    nodes.resize(n_nodes_);
    // nodes[1].makeLeaf();
    MAKE_LEAF(nodes[1]);
    std::cout << "# of nodes: " << nodes.size() << std::endl;

    node_elements.resize(nodes.size());
    std::cout <<"max depth nodes range: " << 1+(std::pow(4,max_depth_)-1)/3 <<"~" <<(std::pow(4,max_depth_+1)-1)/3<<std::endl;

    // Max depth and normalized tree size 
    nodes[1].rect.tl = Pos2d<uint16_t>(0,0);
    nodes[1].rect.br = Pos2d<uint16_t>(std::pow(2, max_depth_),std::pow(2, max_depth_));
    std::cout << "grid size: " << nodes[1].rect <<std::endl;

    // Normalizer
    x_normalizer_ = nodes[1].rect.br.x/(x_range_[1]-x_range_[0]);
    y_normalizer_ = nodes[1].rect.br.y/(y_range_[1]-y_range_[0]);
    std::cout << "xy normalizer: " << x_normalizer_<<"," <<y_normalizer_<<std::endl;
    std::cout << "xy minimum grid size [px]: " << 1./x_normalizer_<<"," <<1./y_normalizer_<<std::endl;
        
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
  
        all_elems_.push_back(QuadElement(id_data,x_nom,y_nom));
        insert_data_.setData(all_elems_.size()-1, x_nom, y_nom);

        this->insertPrivate(1, 0);
    }
    else throw std::runtime_error("Input x,y is out of the quadtree range.");
};

void Quadtree::insertPrivate(ID id_node, uint8_t depth)
{
    QuadNode&              nd = nodes[id_node]; // current node.
    QuadNodeElements& ndelems = node_elements[id_node];
    
    float& x_nom = insert_data_.x_nom;
    float& y_nom = insert_data_.y_nom;
    ID&  id_elem = insert_data_.id_elem;

    if(IS_LEAF(nd)) { // This is a leaf node.
        addDataToNode(id_node, id_elem);

        if(depth != max_depth_){ // nonmax depth.
            int n_elem = getNumElemOfNode(id_node);
            if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                // Make all child to leaf (with no element)
                ID id_child = GET_FIRST_CHILD_ID(id_node);
                MAKE_LEAF(nodes[id_child]);   getQuadrantRect(0, 0, nd.rect, nodes[id_child].rect);
                MAKE_LEAF(nodes[++id_child]); getQuadrantRect(0, 1, nd.rect, nodes[id_child].rect);
                MAKE_LEAF(nodes[++id_child]); getQuadrantRect(1, 0, nd.rect, nodes[id_child].rect);
                MAKE_LEAF(nodes[++id_child]); getQuadrantRect(1, 1, nd.rect, nodes[id_child].rect);

                // Do divide.
                for(auto const& id_elem_tmp : ndelems.elem_ids) {
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
    else if(IS_BRANCH(nd)) {
        // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
        Flag flag_sn, flag_ew;
        FIND_QUADRANT(x_nom, y_nom, nd.rect,      flag_sn, flag_ew);
        ID id_child = GET_CHILD_ID_FLAGS(id_node, flag_sn, flag_ew);
        QuadNode& nd_child = nodes[id_child];

        insertPrivate(id_child, depth + 1); // Go to the child
    }
};

inline void Quadtree::getQuadrant(float x, float y, const QuadRect_u16& qrect, Flag& flag_sn, Flag& flag_ew){
    uint16_t c_x = (qrect.tl.x + qrect.br.x) >> 1;
    uint16_t c_y = (qrect.tl.y + qrect.br.y) >> 1;
    flag_ew = x > c_x; // west : 0, east : 1
    flag_sn = y > c_y; // north: 0, south: 1
};

inline void Quadtree::addDataToNode(ID id_node, ID id_elem){
    node_elements[id_node].elem_ids.push_back(id_elem);
};

inline int Quadtree::getNumElemOfNode(ID id_node){
    return node_elements[id_node].elem_ids.size();
};

inline void Quadtree::getQuadrantRect(
    Flag flag_sn, Flag flag_ew, const QuadRect_u16& qrect,
    QuadRect_u16& qrect_child)
{
    uint16_t cent_x = (qrect.tl.x+qrect.br.x) >> 1;
    uint16_t cent_y = (qrect.tl.y+qrect.br.y) >> 1;
    // (we, ns): (we << 1) + ns; 
    // (0,0): 0 | (1,0): 2
    // (0,1): 1 | (1,1): 3
    qrect_child = qrect;

    if(flag_ew) qrect_child.tl.x = cent_x; // east
    else        qrect_child.br.x = cent_x; // west
    
    if(flag_sn) qrect_child.tl.y = cent_y; // south
    else        qrect_child.br.y = cent_y; // north
};

inline void Quadtree::makeBranch(ID id_node){
    MAKE_BRANCH(nodes[id_node]);
    node_elements[id_node].reset();
};

inline bool Quadtree::inTreeBoundary(float x, float y){
    return INBOUND_RECT(x,y,nodes[1].rect);
};

inline bool Quadtree::inBound(float x, float y, const QuadRect_u16& rect){
    return INBOUND_RECT(x,y,rect);
};

inline bool Quadtree::BWBTest(float x, float y, const QuadRect_u16& rect, float radius){
    // Ball Within Bound (Circle bounded by rect check)
    float rect_size = (float)(rect.br.x-rect.tl.x);
    return (rect_size >= 2*radius) && INBOUND_RECT_PTS(x, y, rect.tl.x+radius,rect.tl.y+radius, rect.br.x-radius, rect.br.y-radius);
};

inline bool Quadtree::BOBTest(float x, float y, const QuadRect_u16& rect, float radius){
    // Ball Overlap Bound (Circle-rect collision check)
    float rect_center_x   = (float)(rect.br.x+rect.tl.x)*0.5;
    float rect_center_y   = (float)(rect.br.y+rect.tl.y)*0.5;
    float rect_halfsize_x = (float)(rect.br.x-rect.tl.x)*0.5;
    float rect_halfsize_y = (float)(rect.br.y-rect.tl.y)*0.5;

    float circle_dist_x = fabs(x - rect_center_x);
    float circle_dist_y = fabs(y - rect_center_y);

    return (circle_dist_x <= rect_halfsize_x + radius)
        && (circle_dist_y <= rect_halfsize_y + radius)
        && (circle_dist_x <= rect_halfsize_x)
        && (circle_dist_y <= rect_halfsize_y)
        && (DIST_EUCLIDEAN(circle_dist_x, circle_dist_y, rect_halfsize_x, rect_halfsize_y) <= radius*radius);

    // float rect_center_x   = (float)(rect.br.x+rect.tl.x)*0.5;
    // float rect_center_y   = (float)(rect.br.y+rect.tl.y)*0.5;
    // float rect_halfsize_x = (float)(rect.br.x-rect.tl.x)*0.5;
    // float rect_halfsize_y = (float)(rect.br.y-rect.tl.y)*0.5;

    // float circle_dist_x = fabs(x - rect_center_x);
    // float circle_dist_y = fabs(y - rect_center_y);

    // if(circle_dist_x > rect_halfsize_x + radius) return false;
    // if(circle_dist_y > rect_halfsize_y + radius) return false;
    
    // if(circle_dist_x <= rect_halfsize_x) return true;
    // if(circle_dist_y <= rect_halfsize_y) return true;
    // float corner_dist_sq = DIST_EUCLIDEAN(circle_dist_x,circle_dist_y,rect_halfsize_x,rect_halfsize_y);
    // return (corner_dist_sq <= radius*radius);
};

void Quadtree::resetNNParameters(){
    // Initialize values
    simple_stack_.clear();
    min_dist2_ = std::numeric_limits<float>::max();
};

void Quadtree::nearestNeighborSearchPrivate(){
    // In this function, search the nearest element from the scratch (from the root node)
    float& x_nom = query_data_.x_nom;
    float& y_nom = query_data_.y_nom;
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

            if( BWBTest(x_nom, y_nom, nd.rect, sqrt(min_dist2_)) ) break; // the nearest point is inside the node.
        }
        else{ // this is not a leaf node.
            // if BOB is not satisfied, dont go to the child
            if( BOBTest(x_nom, y_nom, nd.rect, sqrt(min_dist2_)) ){ // if this node is overlapped by circle,
                // Ball is overlaped to this node.
                simple_stack_.addTotalAccess();

                // Go to child. Find most probable child first.
                ID id_child = GET_FIRST_CHILD_ID(id_node);
                if(IS_ACTIVATED(nodes[id_child]))   simple_stack_.push(id_child);
                if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
                if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
                if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
            }
        }
    }
#ifdef VERBOSE_
    std::cout <<"\n --- statistics - # access nodes: " << simple_stack_.getTotalAccess() << std::endl;
#endif
};



bool Quadtree::findNearestElem(float x, float y, const QuadNodeElements& elems_thisnode){
    bool findNewNearest = false;
    for(auto const& id_elem : elems_thisnode.elem_ids){
        QuadElement& elem = all_elems_[id_elem];
        float dist_temp = DIST_EUCLIDEAN(x,y, elem.x_nom,elem.y_nom);
        if(dist_temp < min_dist2_){
            min_dist2_      = dist_temp;
            this->query_data_.id_data_matched = elem.id_data;
            this->query_data_.id_elem_matched = id_elem;
            this->query_data_.min_dist2_      = dist_temp;
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
        if( BWBTest(x_nom, y_nom, nodes[query_data_.id_node_matched].rect, sqrt(min_dist2_)) ) {
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
            if(BWBTest(x_nom, y_nom, nodes[id_node].rect, sqrt(min_dist2_))) break; 

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

                if( BWBTest(x_nom, y_nom, nd.rect, sqrt(min_dist2_)) ) break; // the nearest point is inside the node.
            }
            else{ // this is not a leaf node.
                // if BOB is not satisfied, dont go to the child
                // if(nd.isActivated() 
                if(IS_ACTIVATED(nd) 
                && BOBTest(x_nom, y_nom, nd.rect, sqrt(min_dist2_)) ){ // if this node is overlapped by circle,
                    // Ball is overlaped to this node.
                    simple_stack_.addTotalAccess();

                    // Go to child. Find most probable child first.
                    ID id_child = GET_FIRST_CHILD_ID(id_node);
                    if(IS_ACTIVATED(nodes[id_child]))   simple_stack_.push(id_child);
                    if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
                    if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
                    if(IS_ACTIVATED(nodes[++id_child])) simple_stack_.push(id_child);
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
ID Quadtree::NNSearch(float x, float y){
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
    return query_data_.id_node_matched;
};

ID Quadtree::NNSearchDebug(float x, float y, uint32_t& n_access){
    ID id_matched = NNSearch(x,y);
    n_access = simple_stack_.getTotalAccess();
    return id_matched;
};

ID Quadtree::cachedNNSearch(float x, float y, int id_node_cached){
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
    return query_data_.id_node_matched;
};

ID Quadtree::cachedNNSearchDebug(float x, float y, int id_node_cached, uint32_t& n_access){
    ID id_matched = cachedNNSearch(x,y,id_node_cached);
    n_access = simple_stack_.getTotalAccess();
    return id_matched;
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
    query_data_.x     = -1.0f;
    query_data_.y     = -1.0f;
    query_data_.x_nom = -1.0f;
    query_data_.y_nom = -1.0f;   
};