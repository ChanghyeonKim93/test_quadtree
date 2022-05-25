#ifndef _QUADTREE2_H_
#define _QUADTREE2_H_
#include <iostream>
#include <cmath>
#include <vector>

#include <list>

// ref: https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det



enum State{
    UNKNOWN = -1,
    PARTIALLY_SEEN = 0,
    FULLY_SEEN = 1
};

struct QuadNode{ // 8 bytes
    // AABB (Axis alinged Bounding box) is not contained, but just 
    // they are just calculated on the fly if needed.
    // This is more efficient because reducing the memory use of a node can 
    // proportionally reduce cache misses when you traverse the tree.
    
    // Points to the first child if this node is a branch or the first
    // element if this node is a leaf.
    int32_t first_child;
    // first_child+0 = index to 1st child (TL)
    // first_child+1 = index to 2nd child (TR)
    // first_child+2 = index to 3nd child (BL)
    // first_child+3 = index to 4th child (BR)


    // Stores the number of elements in the leaf 
    // or -1 if it this node is not a leaf node.
    int32_t count;

    QuadNode() : first_child(-1), count(0) {};
}; 

typedef std::list<QuadNode> QuadNodeList;

struct QuadVertex{
    uint32_t x;
    uint32_t y;

    QuadVertex() : x(0),y(0) { };
    QuadVertex(const uint32_t& x_, const uint32_t& y_) : x(x_), y(y_) { };

    QuadVertex& operator=(const QuadVertex& c){
        x = c.x;
        y = c.y;
        return *this;
    };
    QuadVertex& operator+=(const QuadVertex& c){
        x += c.x;
        y += c.y;
        return *this;
    };
    QuadVertex& operator-=(const QuadVertex& c){
        x -= c.x;
        y -= c.y;
        return *this;
    };
    friend std::ostream& operator<<(std::ostream& os, const QuadVertex& c){
        os << "["<< c.x <<"," << c.y <<"]\n";
        return os;
    };

    void divideHalf(){
        x = x >> 1;
        y = y >> 1;
    };
};

struct QuadRect{
    QuadVertex tl;
    QuadVertex br;
    QuadRect() : tl(0,0), br(0,0) { };

    friend std::ostream& operator<<(std::ostream& os, const QuadRect& c){
        os << "tl:["<< c.tl.x <<"," << c.tl.y <<"],br:[" << c.br.x <<","<<c.br.y<<"]\n";
        return os;
    };
};

struct QuadElement{
    // Stores the ID for the element (can be used to refer to external data).
    int id;

    // Stores the rectangle for the element.
    int x1,y1,x2,y2;
};

struct QuadElementNode{
    // Points to the next element in the leaf node. A value of -1
    // indicates the end of the list.
    int next; 

    // Stores the element index
    int element;
};

class Quadtree{
public:
    Quadtree();
    ~Quadtree();

    // QuadNodeList findAllLeaves();
    // QuadNodeList findLeavesWithinRect(const QuadRect& rect);

private:
    // Stores all the elements in the quadtree.
    std::vector<QuadElement> quad_elements_;

    // Stores all the element nodes in the quadtree.
    std::vector<QuadElementNode> quad_element_nodes_;

private:
    // Stores all the nodes in the quadtree. The first node in this
    // sequence is always the root.
    std::vector<QuadNode> nodes; // 자식 노드가 생기면 4개연속으로 넣는다.

    // |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  
    // | root| tl0 | bl0 | tr0 | br0 |  5  |  6  |  7  |  8  |  9  |  

    QuadRect root_rect; // Stores the quadtree extents.
    
    float x_range_;
    float y_range_;

    float x_normalizer_;
    float y_normalizer_;

    uint32_t max_depth_;
};

#endif