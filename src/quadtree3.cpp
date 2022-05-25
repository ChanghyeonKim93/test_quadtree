#include "quadtree3.h"

Quadtree::Quadtree(
    const float& x_min,
    const float& x_max,
    const float& y_min,
    const float& y_max,
    const uint32_t& max_depth)
: max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max}
{

    // Max depth and normalized tree size 
    root_rect.tl = PosUint32t(0,0);
    root_rect.br = PosUint32t(std::pow(2, max_depth_-1),std::pow(2, max_depth_-1));
    std::cout << root_rect << std::endl;

    // Normalizer
    x_normalizer_ = root_rect.br.x/(x_range_[1]-x_range_[0]);
    y_normalizer_ = root_rect.br.y/(y_range_[1]-y_range_[0]);
    std::cout << "xy normalizer : " << x_normalizer_<<"," <<y_normalizer_<<std::endl;
        
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
        std::cout << "\n======== insert [" << x_nom << "," << y_nom << "] ========\n"; 

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

            std::cout << "[nonmax leaf] ndid: " << id_node <<", depth:" << depth <<", # elem: " << nd.count <<", " <<getNumElemOfNode(id_node) << "\n";
            
            int n_elem = getNumElemOfNode(id_node);
            if(n_elem > max_elem_per_leaf_){ // too much data. divide.
                std::cout <<" --- TOO MUCH DATA... divide! # elem: " << n_elem << "\n";
                
                // Make all child to leaf
                for(int i = 1; i <= 4; ++i) nodes[(id_node << 2)+i].makeLeaf();
                
                // Do divide.
                for(int i = 0; i < ndelems.elems.size(); ++i){
                    QuadElement& elem = ndelems.elems[i];

                    uint32_t id_quadrant = getQuadrant(elem.x_nom, elem.y_nom, qrect);
                    uint32_t id_child = (id_node << 2) + id_quadrant;
                    
                    std::cout <<" --- --- " << i <<"-th elem goes from [" << id_node <<"] to [" << id_child <<"]\n";


                    QuadRect qrect_child = getQuadrantRect(id_quadrant, qrect);
                    insertPrivate(id_child, qrect_child, id_data, elem.x_nom, elem.y_nom, depth + 1);
                }
                //make node branch
                makeBranch(id_node);
                std::cout <<" --- division of [" << id_node << "]: " << nd.count << std::endl;
            }
        }
        else{
            std::cout << "[     branch] ndid: " << id_node <<", depth:" << depth <<"\n";
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
        std::cout << "[   MAX leaf] ndid: " << id_node <<", depth:" << depth <<", # elem: " << nd.count  <<"\n";
    }
};

inline uint32_t Quadtree::getQuadrant(const float& x, const float& y, const QuadRect& qrect){
    float cent_x = (float)((qrect.tl.x+qrect.br.x)>>1);
    float cent_y = (float)((qrect.tl.y+qrect.br.y)>>1);
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

// QuadNodeList Quadtree::findAllLeaves(){
    
//     QuadNodeList leaves, to_process;
//     to_process.push_back(nodes[0]);

//     while(to_process.size() > 0){
//         const QuadNode nd = (const QuadNode)to_process.back();
//         to_process.pop_back();

//         // If this node is a leaf, insert it to the list
//         if(nodes[nd.index].count != -1){ // 
//             leaves.push_back(nd);
//         }
//         else{
//             // Otherwise push the children that intersect the rectangle.

//             nd.
//         }
//     }

//     return leaves;
// };


