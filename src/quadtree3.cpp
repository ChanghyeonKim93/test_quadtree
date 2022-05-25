#include "quadtree3.h"

Quadtree::Quadtree(
    const float& x_min, const float& x_max,
    const float& y_min, const float& y_max,
    const uint32_t& max_depth)
: max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max}
{

    // Max depth and normalized tree size 
    root_rect.tl = PosUint32t(0,0);
    root_rect.br = PosUint32t(std::pow(2, max_depth_-1),std::pow(2, max_depth_-1));
    std::cout << "grid size: " << root_rect <<std::endl;

    // Normalizer
    x_normalizer_ = root_rect.br.x/(x_range_[1]-x_range_[0]);
    y_normalizer_ = root_rect.br.y/(y_range_[1]-y_range_[0]);
    std::cout << "xy normalizer : " << x_normalizer_<<"," <<y_normalizer_<<std::endl;
    std::cout << "xy grid size : " << 1./x_normalizer_<<"," <<1./y_normalizer_<<std::endl;
        
    // Make nodes
    nodes.resize(1*(std::pow(4,max_depth_)-1)/3);
    nodes[0].makeLeaf();

    node_elements.resize(nodes.size());
    std::cout << "# of nodes: " << nodes.size() << std::endl;

    max_elem_per_leaf_ = 2;
};

Quadtree::~Quadtree() { 
    // there is no dynamic memory allocation.
    std::cout << "Quadtree is deleted.\n";
};

void Quadtree::insert(const int32_t& data_id, const float& x, const float& y){
    if( x > x_range_[0] && x < x_range_[1]
    &&  y > y_range_[0] && y < y_range_[1] )
    {
        float x_nom = x*x_normalizer_;
        float y_nom = y*y_normalizer_;
#ifndef VERBOSE_
        std::cout << "\n======== insert [" << x_nom << "," << y_nom << "] ========\n"; 
#endif
        this->insertPrivate(0, root_rect, data_id, x_nom, y_nom, 0);
    }
    else throw std::runtime_error("out of quadtree range.");
};

void Quadtree::insertPrivate(
    const uint32_t& id_node, const QuadRect& qrect, 
    const int& id_data, const float& x, const float& y, uint32_t depth)
{
    QuadNode&          nd = nodes[id_node]; // current node.
    QuadElements& ndelems = node_elements[id_node];
    
    if(depth < max_depth_){ // non-max depth.
        if(nd.isLeaf()) { 
            // This is a leaf node.
            addDataToNode(id_node, id_data, x, y);
#ifndef VERBOSE_
            std::cout << "[nonmax leaf] ndid: " << id_node <<", depth:" << depth <<", # elem: " << nd.count <<", " <<getNumElemOfNode(id_node) << "\n";
#endif

            int n_elem = getNumElemOfNode(id_node);
            if(n_elem > max_elem_per_leaf_){ // too much data. divide.
#ifndef VERBOSE_
                std::cout <<" --- TOO MUCH DATA... divide! # elem: " << n_elem << "\n";
#endif
                // Make all child to leaf
                for(int i = 1; i <= 4; ++i) nodes[(id_node << 2)+i].makeLeaf();
                
                // Do divide.
                for(int i = 0; i < ndelems.elems.size(); ++i){
                    QuadElement& elem = ndelems.elems[i];

                    uint32_t id_quadrant = getQuadrant(elem.x_nom, elem.y_nom, qrect);
                    uint32_t id_child = (id_node << 2) + id_quadrant;
#ifndef VERBOSE_
                    std::cout <<" --- --- " << i <<"-th elem goes from [" << id_node <<"] to [" << id_child <<"]\n";
#endif

                    QuadRect qrect_child = getQuadrantRect(id_quadrant, qrect);
                    insertPrivate(id_child, qrect_child, id_data, elem.x_nom, elem.y_nom, depth + 1);
                }
                //make node branch
                makeBranch(id_node);
#ifndef VERBOSE_
                std::cout <<" --- division of [" << id_node << "]: " << nd.count << std::endl;
#endif
            }
        }
        else{
#ifndef VERBOSE_
            std::cout << "[     branch] ndid: " << id_node <<", depth:" << depth <<"\n";
#endif
            // this might be a branch or uninitialized one.
            // Traverse.
            uint32_t id_quadrant = getQuadrant(x,y,qrect);
            uint32_t id_child = (id_node << 2) + id_quadrant;
            
            QuadRect qrect_child = getQuadrantRect(id_quadrant, qrect);

            // Recursively go...
            insertPrivate(id_child, qrect_child, id_data, x, y, depth+1);
        }
    }
    else { // MAX depth.
        // make this node leaf if not.
        if(!nd.isLeaf()) nd.makeLeaf();
        
        // Add data_id 
        addDataToNode(id_node, id_data, x, y);
#ifndef VERBOSE_
        std::cout << "[   MAX leaf] ndid: " << id_node <<", depth:" << depth <<", # elem: " << nd.count  <<"\n";
#endif
    }
};

inline uint32_t Quadtree::getQuadrant(const float& x, const float& y, const QuadRect& qrect){
    float cent_x = (float)((qrect.tl.x + qrect.br.x) >> 1);
    float cent_y = (float)((qrect.tl.y + qrect.br.y) >> 1);
    if(x < cent_x) {
        if(y < cent_y) return 1; // 1 (tl)
        else return 2; // 2 (bl)
    }
    else {
        if(y < cent_y) return 3; // 3 (tr)
        else return 4; // 4 (br)
    }
};

inline void Quadtree::addDataToNode(const uint32_t& id_node, const int& id_data, const float& x_nom, const float& y_nom){
    node_elements[id_node].elems.push_back(QuadElement(id_data,x_nom,y_nom));
    ++nodes[id_node].count;
};

inline int Quadtree::getNumElemOfNode(const uint32_t& id_node){
    return node_elements[id_node].elems.size();
};

inline QuadRect Quadtree::getQuadrantRect(const uint32_t& id_quadrant, const QuadRect& qrect){
    QuadRect qrect_child;
    uint32_t cent_x = (uint32_t)((qrect.tl.x+qrect.br.x)>>1);
    uint32_t cent_y = (uint32_t)((qrect.tl.y+qrect.br.y)>>1);
    
    switch(id_quadrant){
        case 1:
            qrect_child.tl = qrect.tl;
            qrect_child.br = PosUint32t(cent_x,     cent_y);
            break;
        case 2:
            qrect_child.tl = PosUint32t(qrect.tl.x, cent_y);
            qrect_child.br = PosUint32t(cent_x, qrect.br.y);
            break;
        case 3:
            qrect_child.tl = PosUint32t(cent_x, qrect.tl.y);
            qrect_child.br = PosUint32t(qrect.br.x, cent_y);
            break;
        case 4:
            qrect_child.tl = PosUint32t(cent_x,     cent_y);
            qrect_child.br = qrect.br;
            break;
        default:
            throw std::runtime_error("switch is failed.");
            break;
    }
    return qrect_child;
};

inline void Quadtree::makeBranch(const uint32_t& id_node){
    nodes[id_node].makeBranch();
    node_elements[id_node].reset();
};


inline uint32_t Quadtree::getParentID(const uint32_t& id_node){
    return (id_node >> 2);
};

inline uint32_t Quadtree::getMyQuadrant(const uint32_t& id_node){
    return (id_node % 4);
};


/*
    NNSearch(0, x, y);

    while(stack.size() > 0){
        id_node = stack.top();
        stack.pop();
        QuadNode& nd = nodes[id_node];
        QuadElements& nd_elems = node_elements[id_node];
        // If leaf node, find nearest point
        if(nd.isLeaf()){
            ++total_access;
            findNearestElem(nd_elems, x, y, id_node_matched);
            if( BWBTest(nd, x, y) ) break; // the nearest point is inside the node.
        }
        else{ // this is not a leaf node.
            // if BOB is not satisfied, dont go to the child
            if( BOBTest(nd, x, y)){
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

    stack.clear();
    return id_elem_matched;

*/

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