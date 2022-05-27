#ifndef _QUADTREE_FAST_H_
#define _QUADTREE_FAST_H_
#include <iostream>
#include <vector>
#include <cmath>
#include "simple_stack.h"
#include "macros.h"

// #define VERBOSE_

typedef uint32_t ID; // 4 bytes
typedef uint8_t  Flag; // 1 bytes

class Quadtree{
private:
    template <typename T>
    struct Pos2d{ // sizeof(T)*2 bytes
        T x;
        T y;

        Pos2d() : x(0),y(0) { };
        Pos2d(T x_, T y_) : x(x_), y(y_) { };
        Pos2d(const Pos2d& pos) : x(pos.x),y(pos.y) { }; // copy constructor

        Pos2d& operator=(const Pos2d& c){ // copy insert constructor
            x = c.x; y = c.y;
            return *this;
        };
        Pos2d& operator+=(const Pos2d& c){
            x += c.x; y += c.y;
            return *this;
        };
        Pos2d& operator-=(const Pos2d& c){
            x -= c.x; y -= c.y;
            return *this;
        };
        friend std::ostream& operator<<(std::ostream& os, const Pos2d& c){
            os << "["<< c.x <<"," << c.y <<"]";
            return os;
        };

    };

    template <typename T>
    struct QuadRect{ // sizeof(T)*4 bytes
        Pos2d<T> tl;
        Pos2d<T> br;
        QuadRect() : tl(0,0), br(0,0) { };
        QuadRect(const Pos2d<T>& tl_, const Pos2d<T>& br_) : tl(tl_), br(br_) { };

        inline Pos2d<T> getCenter(){
            return Pos2d<T>((tl.x+br.x)*0.5, (tl.y+br.y)*0.5);
        };

        friend std::ostream& operator<<(std::ostream& os, const QuadRect& c){
            os << "tl:["<< c.tl.x <<"," << c.tl.y <<"],br:[" << c.br.x <<","<<c.br.y<<"]";
            return os;
        };
    };

    typedef QuadRect<uint16_t> QuadRect_u16;
    typedef QuadRect<float>    QuadRect_f;

    struct QuadNode{ // 10 bytes (actually 10 bytes)
        // AABB (Axis-alinged Bounding box) is not contained, but just 
        // they are just calculated on the fly if needed.
        // This is more efficient because reducing the memory use of a node can 
        // proportionally reduce cache misses when you traverse the tree.
        QuadRect_u16 rect; // 2 * 4  = 8 bytes (padding size = 2 bytes)

        // If -2, not initialized (no children.) 
        // else if -1,  branch node. (children exist.)
        // else, leaf node.
        uint8_t state; // 1 byte (-2 : unactivated, -1: branch, 0: leaf)
        int8_t depth; // 1 byte

        QuadNode() : state(1), depth(-1) {};
        friend std::ostream& operator<<(std::ostream& os, const QuadNode& c){
            os << "count:[" << c.state <<"]";
            return os;
        };

#define STATE_UNACTIVATED 0b0001 // 1
#define STATE_BRANCH      0b0010 // 2 
#define STATE_LEAF        0b0100 // 4
#define STATE_ACTIVATED   0b0110 // 6

#define IS_ACTIVATED(nd) ((nd).state | STATE_ACTIVATED)
#define IS_BRANCH(nd)    ((nd).state & STATE_BRANCH)
#define IS_LEAF(nd)      ((nd).state & STATE_LEAF)
#define MAKE_BRANCH(nd)  ((nd).state = STATE_BRANCH)
#define MAKE_LEAF(nd)    ((nd).state = STATE_LEAF)

        inline bool isActivated() { return (state > 1); };
        inline bool isBranch()   { return (state == 2); };
        inline bool isLeaf()     { return (state == 4); };
        inline void makeBranch() { state = 2; };
        inline void makeLeaf()   { state = 4; };
    }; 

    struct QuadElement{ // 12 bytes (actually 16 bytes)
        ID id_data; // 4 bytes
        float  x_nom;   // 4 bytes
        float  y_nom;   // 4 bytes

        QuadElement() : id_data(0),x_nom(-1.0f),y_nom(-1.0f) { };
        QuadElement(int id_data_, float x_nom_, float y_nom_)
        : id_data(id_data_), x_nom(x_nom_), y_nom(y_nom_) { };
    };

    struct QuadNodeElements{ // 8 bytes
        // Stores the ID for the element (can be used to refer to external data).
        std::vector<ID> elem_ids; // 8 bytes

        QuadNodeElements() { elem_ids.resize(0); };
        inline void reset() { elem_ids.resize(0); };
        inline int getNumElem() const { return elem_ids.size(); };
    };

    struct InsertData{ // 12 bytes (actually 16 bytes)
        float x_nom; // 4 bytes
        float y_nom; // 4 bytes
        ID id_elem;  // 4 bytes

        InsertData() : x_nom(-1.0f), y_nom(-1.0f), id_elem(0){};
        void setData(ID id_elem_, float x_nom_, float y_nom_){
            x_nom   = x_nom_; 
            y_nom   = y_nom_;
            id_elem = id_elem_;
        };
    };

    struct QuaryData{
        float x; // 4 bytes
        float y; // 4 bytes
        float x_nom; // 4 bytes
        float y_nom; // 4 bytes
        ID id_node_cached; // 4 bytes

        ID id_data_matched; // 4 bytes
        ID id_elem_matched; // 4 bytes
        ID id_node_matched; // 4 bytes

        float min_dist2_;
    };

public:
    Quadtree(float x_min, float x_max, float y_min, float y_max, uint32_t max_depth, uint32_t max_elem_per_leaf);
    ~Quadtree();

    void insert(float x, float y, int id_data);
    ID NNSearch(float x, float y);
    ID cachedNNSearch(float x, float y, int id_node_cached);
    
    ID NNSearchDebug(float x, float y, uint32_t& n_access);
    ID cachedNNSearchDebug(float x, float y, int id_node_cached, uint32_t& n_access);

// Related to generate tree.
private:
    void insertPrivate(ID id_node, uint8_t depth);
    inline void getQuadrant(float x, float y, const QuadRect_u16& qrect, Flag& flag_we, Flag& flag_ns);
    inline QuadRect_u16 getQuadrantRect(Flag flag_we, Flag flag_ns, const QuadRect_u16& qrect);

    inline void addDataToNode(ID id_node, ID id_elem);
    inline int getNumElemOfNode(ID id_node);

    inline void makeBranch(ID id_node);

// Related to NN search (private)
private:
    void nearestNeighborSearchPrivate(); // return id_data matched.
    
    inline bool inTreeBoundary(float x, float y);
    inline bool inBound( float x, float y, const QuadRect_u16& rect);
    inline bool BWBTest( float x, float y, const QuadRect_u16& rect, float radius); // Ball Within Bound
    inline bool BOBTest( float x, float y, const QuadRect_u16& rect, float radius); // Ball Overlap Bound
    bool findNearestElem(float x, float y, const QuadNodeElements& elems);

// Related to cached NN search (private)
private:
    void cachedNearestNeighborSearchPrivate();

private:
    // Stores all the elements in the quadtree.
    std::vector<QuadElement>  all_elems_;
    std::vector<QuadNodeElements> node_elements;

private:
    // Stores all the nodes in the quadtree. The second node in this
    // sequence is always the root.
    // index 0 is not used.
    std::vector<QuadNode> nodes;
    uint32_t n_nodes_;
    // |  1  |  2  |  3  |  4  |  5  |  ...
    // | root| tl0 | bl0 | tr0 | br0 |  ...
    // Z-order
    //  id_node * 4  - 2 -> first child id (child 0~3)
    // (id_node + 2) / 4 -> parent id
    // (id_node + 2) % 4 -> quadrant index of this node

    float x_normalizer_;
    float y_normalizer_;

private: 
    // quadtree range (in real scale)
    float x_range_[2]; float y_range_[2];

    uint32_t max_depth_; // Quadtree maximum depth
    uint32_t max_elem_per_leaf_; // The maximum number of elements in the leaf. If maxdepth leaf, no limit to store elements.

// For insert a data
private:
    InsertData insert_data_;
    inline void resetInsertData();

// For nearest neighbor search algorithm
private:
    QuaryData query_data_;
    float min_dist2_;
    SimpleStack<ID> simple_stack_;

    inline void resetNNParameters();
    inline void resetQueryData();
};

#endif