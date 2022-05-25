#ifndef _QUADTREE3_H_
#define _QUADTREE3_H_
#include <iostream>
#include <cmath>
#include <vector>

enum State{
    UNKNOWN = -1,
    PARTIALLY_SEEN = 0,
    FULLY_SEEN = 1
};

struct PosUint32t{
    uint32_t x;
    uint32_t y;

    PosUint32t() : x(0),y(0) { };
    PosUint32t(const uint32_t& x_, const uint32_t& y_) : x(x_), y(y_) { };
    PosUint32t(const PosUint32t& pos) : x(pos.x),y(pos.y) { };

    PosUint32t& operator=(const PosUint32t& c){
        x = c.x;
        y = c.y;
        return *this;
    };
    PosUint32t& operator+=(const PosUint32t& c){
        x += c.x;
        y += c.y;
        return *this;
    };
    PosUint32t& operator-=(const PosUint32t& c){
        x -= c.x;
        y -= c.y;
        return *this;
    };
    friend std::ostream& operator<<(std::ostream& os, const PosUint32t& c){
        os << "["<< c.x <<"," << c.y <<"]\n";
        return os;
    };

};

struct QuadRect{
    PosUint32t tl;
    PosUint32t br;
    QuadRect() : tl(0,0), br(0,0) { };
    QuadRect(const PosUint32t& tl_, const PosUint32t& br_) : tl(tl_), br(br_) { };

    inline PosUint32t getCenter(){
        return PosUint32t((tl.x+br.x)>>1, (tl.y+br.y)>>1);
    };

    friend std::ostream& operator<<(std::ostream& os, const QuadRect& c){
        os << "tl:["<< c.tl.x <<"," << c.tl.y <<"],br:[" << c.br.x <<","<<c.br.y<<"]\n";
        return os;
    };
};


struct QuadNode{ // 8 bytes
    // AABB (Axis alinged Bounding box) is not contained, but just 
    // they are just calculated on the fly if needed.
    // This is more efficient because reducing the memory use of a node can 
    // proportionally reduce cache misses when you traverse the tree.
    
    // Points to the first child if this node is a branch or the first
    // element if this node is a leaf.
    // int32_t first_child;
    // first_child+0 = index to 1st child (TL)
    // first_child+1 = index to 2nd child (TR)
    // first_child+2 = index to 3nd child (BL)
    // first_child+3 = index to 4th child (BR)

    // Element node id. 
    // If -2, not initialized (no children.) 
    // else if -1,  branch node. (children exist.)
    // else, leaf node.
    int32_t count;

    QuadNode() : count(-2) {};
    friend std::ostream& operator<<(std::ostream& os, const QuadNode& c){
        os << "count:[" << c.count <<"]\n";
        return os;
    };

    inline bool isLeaf()     { return (count >  -1); };
    inline bool isBranch()   { return (count == -1); };
    inline void makeLeaf()   { count =  0; };
    inline void makeBranch() { count = -1; };
}; 
struct QuadElement{
    int data_id;
    float x_nom;
    float y_nom;
    QuadElement() : data_id(-1),x_nom(-1),y_nom(-1) { };
    QuadElement(const int& data_id_, const float& x_nom_, const float& y_nom_)
    : data_id(data_id_), x_nom(x_nom_), y_nom(y_nom_) { };
};
struct QuadElements{
    // Stores the ID for the element (can be used to refer to external data).
    std::vector<QuadElement> elems;

    QuadElements() { elems.resize(0); };

    inline void reset() { elems.resize(0); };
    inline int getNumElem() const { return elems.size(); };
};


class Quadtree{
public:
    Quadtree(
    const float& x_min,
    const float& x_max,
    const float& y_min,
    const float& y_max,
    const uint32_t& max_depth);
    ~Quadtree();

    // QuadNodeList findAllLeaves();
    // QuadNodeList findLeavesWithinRect(const QuadRect& rect);

    void insert(const int32_t& data_id, const float& x, const float& y);

private:
    void insertPrivate(const uint32_t& id_node, const QuadRect& qrect, const int& id_data, const float& x, const float& y, uint32_t depth);
    inline uint32_t getQuadrant(const float& x, const float& y, const QuadRect& qrect);
    inline QuadRect getQuadrantRect(const uint32_t& id_quadrant, const QuadRect& qrect);

    inline void addDataToNode(const uint32_t& id_node, const int& id_data, const float& x_nom, const float& y_nom);
    inline int getNumElemOfNode(const uint32_t& id_node);

    inline void makeBranch(const uint32_t& id_node);

private:
    // Stores all the elements in the quadtree.
    std::vector<QuadElements> node_elements;

private:
    // Stores all the nodes in the quadtree. The first node in this
    // sequence is always the root.
    std::vector<QuadNode> nodes;
    // |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  
    // | root| tl0 | bl0 | tr0 | br0 |  5  |  6  |  7  |  8  |  9  |  
    // Z-order

    QuadRect root_rect; // Stores the quadtree extents.
    
    float x_range_[2];
    float y_range_[2];

    float x_normalizer_;
    float y_normalizer_;

    uint32_t max_depth_;
    uint32_t max_elem_per_leaf_;
};

#endif