#include "quadtree_fast.h"

Quadtree::Quadtree(
    float x_min, float x_max,
    float y_min, float y_max,
    uint32_t max_depth)
: max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max}
{
    // Make nodes
    nodes.resize(1 + 1*(std::pow(4,max_depth_)-1)/(4-1));
    nodes[1].makeLeaf();
    std::cout << "# of nodes: " << nodes.size() << std::endl;

    node_elements.resize(nodes.size());

    // Max depth and normalized tree size 
    nodes[1].rect.tl = Pos2d<uint16_t>(0,0);
    nodes[1].rect.br = Pos2d<uint16_t>(std::pow(2, max_depth_-1),std::pow(2, max_depth_-1));
    std::cout << "grid size: " << nodes[1].rect <<std::endl;

    // Normalizer
    x_normalizer_ = nodes[1].rect.br.x/(x_range_[1]-x_range_[0]);
    y_normalizer_ = nodes[1].rect.br.y/(y_range_[1]-y_range_[0]);
    std::cout << "xy normalizer: " << x_normalizer_<<"," <<y_normalizer_<<std::endl;
    std::cout << "xy minimum grid size [px]: " << 1./x_normalizer_<<"," <<1./y_normalizer_<<std::endl;
        
    max_elem_per_leaf_ = 1;

    std::cout <<"sizeof QuadNode: " << sizeof(Quadtree::QuadNode) << std::endl;

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
#ifndef VERBOSE_
        std::cout << "\n======== insert [" << x_nom << "," << y_nom << "] ========\n"; 
#endif
  
        all_elems_.push_back(QuadElement(id_data,x_nom,y_nom));
        insert_data_.setData(all_elems_.size()-1, x_nom, y_nom);

        this->insertPrivate(1, 0);
    }
    else throw std::runtime_error("Input x,y is out of the quadtree range.");
};

void Quadtree::insertPrivate(
    ID id_node, uint8_t depth)
{
    QuadNode&              nd = nodes[id_node]; // current node.
    QuadNodeElements& ndelems = node_elements[id_node];
    
    float& x_nom = insert_data_.x_nom;
    float& y_nom = insert_data_.y_nom;
    ID&  id_elem = insert_data_.id_elem;

    if(depth < max_depth_){ // non-max depth.
        if(nd.isLeaf()) { // This is a leaf node.
            addDataToNode(id_node, x_nom, y_nom, id_elem);

            int n_elem = getNumElemOfNode(id_node);
            if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                // Make all child to leaf (with no element)
                for(uint8_t i = 0; i < 4; ++i) {
                    ID id_child = GET_CHILD_ID_INDEX(id_node, i);
                    QuadNode& nd_child = nodes[id_child];
                    nd_child.makeLeaf();
                    nd_child.rect = getQuadrantRect((i & BITMASK_EAST), i & BITMASK_SOUTH, nd.rect);
                }
                

                // Do divide.
                for(int i = 0; i < ndelems.getNumElem(); ++i){
                    Flag flag_we, flag_ns;
                    ID& id_elem_tmp = ndelems.elem_ids[i];
                    QuadElement& elem_tmp = all_elems_[id_elem_tmp];
                    getQuadrant(elem_tmp.x_nom, elem_tmp.y_nom, nd.rect,
                        flag_we, flag_ns);
                    ID id_child = GET_CHILD_ID_FLAGS(id_node, flag_we,flag_ns);

                    node_elements[id_child].elem_ids.push_back(id_elem_tmp);
                }

                //make this node as a branch
                makeBranch(id_node);
            }
        }
        else if(nd.isBranch()) {
            // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
            uint8_t flag_we, flag_ns;
            getQuadrant(insert_data_.x_nom, insert_data_.y_nom, nd.rect, 
                flag_we, flag_ns);

            ID id_child = GET_CHILD_ID_FLAGS(id_node,flag_we,flag_ns);
            QuadRect rect_child = getQuadrantRect(flag_we, flag_ns, nd.rect);

            // Go to the child
            insertPrivate(id_child, depth+1);
        }
        else { // not activated
            // Activate it as a 'empty leaf'.
            nd.makeLeaf();
            addDataToNode(id_node, x_nom, y_nom, id_elem);
        }
    }
    else { // MAX depth.
        // make this node leaf if not.
        if(!nd.isLeaf()) nd.makeLeaf();
        
        // Add data_id 
        addDataToNode(id_node, x_nom, y_nom, id_elem);
    }
};

inline void Quadtree::getQuadrant(float x, float y, const QuadRect_u16& qrect, Flag& flag_we, Flag& flag_ns){
    // (we, ns): (we << 1) + ns; 
    // (0,0): 0 | (1,0): 2
    // (0,1): 1 | (1,1): 3
    uint16_t c_x = (qrect.tl.x + qrect.br.x) >> 1;
    uint16_t c_y = (qrect.tl.y + qrect.br.y) >> 1;
    flag_we = x > c_x; // west : 0, east : 1
    flag_ns = y > c_y; // north: 0, south: 1
};

inline void Quadtree::addDataToNode(ID id_node, float x_nom, float y_nom, ID id_elem){
    node_elements[id_node].elem_ids.push_back(id_elem);
    ++nodes[id_node].count;
};

inline int Quadtree::getNumElemOfNode(ID id_node){
    return node_elements[id_node].elem_ids.size();
};

inline Quadtree::QuadRect_u16 Quadtree::getQuadrantRect(
    Flag flag_we, Flag flag_ns, const QuadRect_u16& qrect)
{
    QuadRect_u16 qrect_child;
    uint16_t cent_x = (qrect.tl.x+qrect.br.x) >> 1;
    uint16_t cent_y = (qrect.tl.y+qrect.br.y) >> 1;
    // (we, ns): (we << 1) + ns; 
    // (0,0): 0 | (1,0): 2
    // (0,1): 1 | (1,1): 3
    qrect_child = qrect;

    if(flag_we) qrect_child.tl.x = cent_x; // east
    else        qrect_child.br.x = cent_x; // west
    
    if(flag_ns) qrect_child.tl.y = cent_y; // south
    else        qrect_child.br.y = cent_y; // north

    return qrect_child;
};

inline void Quadtree::makeBranch(ID id_node){
    nodes[id_node].makeBranch();
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
    float rect_size = (float)(rect.tl.x-rect.br.x);
    if(rect_size > radius){
        QuadRect_u16 rect_shrink;
        rect_shrink       = rect;
        rect_shrink.tl.x += radius; rect_shrink.tl.y += radius;
        rect_shrink.br.x -= radius; rect_shrink.br.y -= radius;
        return INBOUND_RECT(x, y, rect_shrink);
    }
    else return false;
};

inline bool Quadtree::BOBTest(float x, float y, const QuadRect_u16& rect, float radius){
    // Ball Overlap Bound (Circle-rect collision check)
    Pos2d rect_center((rect.tl.x+rect.br.x)*0.5,(rect.tl.y+rect.br.y)*0.5);
    Pos2d rect_halfsize((rect.br.x-rect.tl.x*0.5),(rect.br.y-rect.tl.y)*0.5);
    
    float circle_dist_x = fabs(x - (float)rect_center.x);
    float circle_dist_y = fabs(y - (float)rect_center.y);

    if(circle_dist_x > rect_halfsize.x + radius) return false;
    if(circle_dist_y > rect_halfsize.y + radius) return false;
    
    if(circle_dist_x <= rect_halfsize.x) return true;
    if(circle_dist_y <= rect_halfsize.y) return true;

    float corner_dist_sq = DIST_EUCLIDEAN(circle_dist_x,circle_dist_y,rect_halfsize.x,rect_halfsize.y);
    return (corner_dist_sq <= radius*radius);
};

void Quadtree::resetNNParameters(){
    // Initialize values
    simple_stack_.clear();
    min_dist2_ = std::numeric_limits<float>::max();
};

void Quadtree::NearestNeighborSearchPrivate(){
    // return id_data matched
    // In this function, search the nearest element from the scratch (from the root node)
    float& x_nom = query_data_.x_nom;
    float& y_nom = query_data_.y_nom;
    ID& id_data_matched = query_data_.id_data_matched;
    ID& id_elem_matched = query_data_.id_elem_matched;
    ID& id_node_matched = query_data_.id_node_matched;

    // Initialize NN parameters
    resetNNParameters();

    simple_stack_.push(1); // root node ID
    while(!simple_stack_.empty()){
        ID id_node = simple_stack_.topAndPop();
        QuadNode&     nd           = nodes[id_node];
        QuadNodeElements& nd_elems = node_elements[id_node];

        // If leaf node, find nearest point and 'BWBTest()'
        if(nd.isLeaf()){
            simple_stack_.addTotalAccess();
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
                if(nodes[id_child].isActivated())   simple_stack_.push(id_child);
                if(nodes[++id_child].isActivated()) simple_stack_.push(id_child);
                if(nodes[++id_child].isActivated()) simple_stack_.push(id_child);
                if(nodes[++id_child].isActivated()) simple_stack_.push(id_child);
            }
        }
    }
    std::cout <<" statistics - # access nodes: " << simple_stack_.getTotalAccess() << std::endl;
};

bool Quadtree::findNearestElem(float x, float y, const QuadNodeElements& elems_thisnode){
    bool findNewNearest = false;
    for(auto id_elem : elems_thisnode.elem_ids){
        QuadElement& elem = all_elems_[id_elem];
        float dist_temp = DIST_EUCLIDEAN(x,y, elem.x_nom,elem.y_nom);
        if(dist_temp < min_dist2_){
            min_dist2_      = dist_temp;
            this->query_data_.id_data_matched = elem.id_data;
            this->query_data_.id_elem_matched = id_elem;
            this->query_data_.min_dist2_      = dist_temp;
            findNewNearest  = true;
            std::cout <<"min dist2 updated: " << dist_temp <<", id_data:"<< elem.id_data <<", id_elem:"<<id_elem <<", xy[" << x <<"," << y<< "]" << ", xyelem[" << elem.x_nom <<"," << elem.y_nom<< "]" << std::endl; 
        }
    }
    return findNewNearest;
};


// Find nearest element.
// return : uint32_t id_data_matched.
ID Quadtree::NNSearch(float x, float y){
    query_data_.x = x;
    query_data_.y = y;
    query_data_.x_nom = x*x_normalizer_;
    query_data_.y_nom = y*y_normalizer_;
    // query_data_.id_node_cached = 0; // non-cached case now.
    
    NearestNeighborSearchPrivate();

    std::cout << "Query data: " << query_data_.x << "," << query_data_.y << std::endl;
    std::cout << "Matched   : " << all_elems_[query_data_.id_elem_matched].x_nom/x_normalizer_
                            <<"," << all_elems_[query_data_.id_elem_matched].y_nom/y_normalizer_ << std::endl;

    return query_data_.id_data_matched;
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

/*
    NNSearchCached(id_node, x, y);
    QuadNode& nd = nodes[id_node]; // start node. (cached)

    if(id_node == 0) NNSearch(x,y);
    else{
        // Find nearest point in the cached node.
        findNearestElem(nd_elems, x, y, id_node_matched);
        if( BWBTest(nd,x,y) ) break;
    }

    // Go up to the BWBTest == true
    id_node = getParentNodeID(id_node);
    while(true){
        if(inBound(node,x,y)) break;

        if(id_node == 0) break; // Root
        id_node = getParentNodeID(id_node);
        ++total_access;
    }

    // From the current node, search !
    stack.push(id_node);
    while(stack.size() > 0){
        nd = nodes[stack.top()];
        stack.pop();

        if(nd.isLeaf()){
            ++total_access;
            findNearestElem();
            if(inBound(nd, x,y) && BWBTest(nd, x, y)) goto finish;
        }
        else{
            if(BOBTest(nd, x,y)){
                ++total_access;
                // Ball is overlaped to this node.

                // Go to child. Find most probable child first.
                for(int i = 1; i <= 4; ++i){
                    id_child = (id_node << 2) + i;
                    stack.push(id_child);
                }
            }
        }
    }

finish:
    stack.clear();
    return id_elem_matched;

*/


/*
    BWBTest(id_node,x,y){
        if(id_node == 0) return true;

        double d_hori, d_vert;
        if (nd_->bound.nw.u == 0)
            d_hori = (double)nd_->bound.se.u - pt_q_.u;
        else if (nd_->bound.se.u == this->width)
            d_hori = pt_q_.u - (double)nd_->bound.nw.u;
        else
            d_hori = (double)nd_->bound.se.u - pt_q_.u < pt_q_.u - (double)nd_->bound.nw.u
            ? (double)nd_->bound.se.u - pt_q_.u : pt_q_.u - (double)nd_->bound.nw.u;

        if (nd_->bound.nw.v == 0)
            d_vert = nd_->bound.se.v - pt_q_.v;
        else if (nd_->bound.se.v == this->height)
            d_vert = pt_q_.v - nd_->bound.nw.v;
        else
            d_vert = (double)nd_->bound.se.v - pt_q_.v < pt_q_.v - (double)nd_->bound.nw.v
            ? (double)nd_->bound.se.v - pt_q_.v : pt_q_.v - (double)nd_->bound.nw.v;

        double d_min = d_hori < d_vert ? d_hori : d_vert;
        // std::cout << "a,b,c,d: " << d_a << ", " << d_b << ", " << d_c << ", " << d_d << std::endl;
        return (*this->min_dist*scale2 < d_min*d_min);
    }
*/


/*
bool QuadTreeFastPooled::BOBTest(Node*& nd_, Point2<double>& pt_q_) {
	// 좌
	double min_dist_scale = *this->min_dist*scale2;
	if (pt_q_.u < nd_->bound.nw.u)
		// 좌상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 좌중
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u);
	// 좌하
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// 중
	else if (pt_q_.u < nd_->bound.se.u)
		// 중상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 중중은 없다.
		else if (pt_q_.v < nd_->bound.se.v)
			return true;
	// 중하
		else
			return min_dist_scale >
			(pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// 우
	else
		// 우상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 우중
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u);
	// 우하
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
};

*/