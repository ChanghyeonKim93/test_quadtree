#ifndef _QUADTREE_POINTER2_H_
#define _QUADTREE_POINTER2_H_
#include <iostream>
#include <vector>
#include <cmath>
#include "quadtree/macro_quadtree.h"
#include "quadtree/simple_stack.h"
#include "quadtree/object_pool.h"

namespace PointerBased2{
    typedef uint32_t ID; // 4 bytes
    typedef uint8_t  Flag; // 1 bytes
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
            return Pos2d<T>((double)(tl.x+br.x)*0.5, (double)(tl.y+br.y)*0.5);
        };

        friend std::ostream& operator<<(std::ostream& os, const QuadRect& c){
            os << "tl:["<< c.tl.x <<"," << c.tl.y <<"],br:[" << c.br.x <<","<<c.br.y<<"]";
            return os;
        };
    };

    typedef uint16_t QuadUint;
    typedef QuadRect<QuadUint> QuadRect_u;
    typedef QuadRect<float>    QuadRect_f;

    struct Elem;
    typedef Elem* ElemPtr;
    struct Elem{ // 20 bytes (actually 16 bytes)
        ElemPtr next;
        float   x_nom;   // 4 bytes
        float   y_nom;   // 4 bytes
        ID      id_data; // 4 bytes, (external index)

        Elem() : next(nullptr), id_data(0), x_nom(-1.0f), y_nom(-1.0f) { };
        Elem(float x_nom_, float y_nom_, int id_data_)
            : next(nullptr), id_data(id_data_), x_nom(x_nom_), y_nom(y_nom_) { };
        inline void resetAndSetData(float x_nom_, float y_nom_, int id_data_){
            next = nullptr;
            x_nom = x_nom_;
            y_nom = y_nom_;
            id_data = id_data_;
        };
    };

    struct QuadNode;
    typedef QuadNode* QuadNodePtr;
    struct QuadNode{ // 46 bytes
        QuadRect_u  rect; // 2 * 4 (8)
        ElemPtr     head; // 8 bytes
        ElemPtr     tail; // 8 bytes
        QuadNodePtr parent; // 8
        QuadNodePtr first_child; // 8
        uint8_t     state; // 1
        int8_t      depth; // 1
        uint32_t    n_elem; // 4 byte 

#define STATE_UNACTIVATED 0b0000 // 0
#define STATE_ACTIVATED   0b0001 // 1 (0b0001)
#define STATE_BRANCH      0b0011 // 3 (0b0011)
#define STATE_LEAF        0b0101 // 5 (0b0101)

#define IS_UNACTIVATED_P(nd) ( (nd)->state == 0b0000         )
#define IS_ACTIVATED_P(nd)   ( (nd)->state  & STATE_ACTIVATED)
#define IS_BRANCH_P(nd)      ( (nd)->state == STATE_BRANCH   )
#define IS_LEAF_P(nd)        ( (nd)->state == STATE_LEAF     )

#define MAKE_UNACTIVATE_P(nd){ (nd)->state = STATE_UNACTIVATED; (nd)->n_elem = 0;}
#define MAKE_ACTIVATE_P(nd)  ( (nd)->state = STATE_ACTIVATED  )
#define MAKE_BRANCH_P(nd)    { (nd)->state = STATE_BRANCH; (nd)->n_elem = 0; }
#define MAKE_LEAF_P(nd)      ( (nd)->state = STATE_LEAF       )

        QuadNode() 
        : state(STATE_UNACTIVATED), head(nullptr), tail(nullptr),
        depth(-1), parent(nullptr), first_child(nullptr), n_elem(0) {};
        friend std::ostream& operator<<(std::ostream& os, const QuadNode& c){
            os << "count:[" << c.state <<"]";
            return os;
        };

        inline void reset(){
            head = nullptr;
            tail = nullptr;
            rect.tl.x = 0; rect.tl.y = 0; rect.br.x = 0; rect.br.y = 0;
            parent = nullptr;
            first_child = nullptr;
            state = STATE_UNACTIVATED;
            depth = -1;
            n_elem = 0;
        };
    };

    class Quadtree{
    private:
        struct InsertData{ // 16 bytes (actually 16 bytes)
            float x_nom;   // 4 bytes
            float y_nom;   // 4 bytes
            ElemPtr elem;    // 8 bytes

            InsertData() : x_nom(-1.0f), y_nom(-1.0f), elem(nullptr){};
            void resetAndSetData(float x_nom_, float y_nom_, ElemPtr elem_){
                x_nom = x_nom_; 
                y_nom = y_nom_;
                elem  = elem_;
            };
        };

        struct QuaryData{
            float x; // 4 bytes
            float y; // 4 bytes
            float x_nom; // 4 bytes
            float y_nom; // 4 bytes

            QuadNodePtr ptr_node_cached; // 4 bytes

            ID          id_data_matched; // 4 bytes
            QuadNodePtr ptr_node_matched; // 4 bytes

            float min_dist2_;
            float min_dist_;
        };

        struct QuadParams{
            // Distance parameter
            float approx_rate; // 0.3~1.0;
            
            // Searching parameter
            uint8_t flag_adj_search_only;
        };

    public:
        Quadtree(float x_min, float x_max, float y_min, float y_max, 
            std::shared_ptr<ObjectPool<QuadNode>> objpool_node, std::shared_ptr<ObjectPool<Elem>> objpool_elem,
            uint32_t max_depth, uint32_t max_elem_per_leaf,
            float approx_rate = 1.0, uint8_t flag_adj = false);
        ~Quadtree();

        void insert(float x, float y, int id_data);
        void NNSearch(float x, float y,
            ID& id_data_matched, QuadNodePtr& ptr_node_matched);
        void cachedNNSearch(float x, float y, QuadNodePtr ptr_node_cached, 
            ID& id_data_matched, QuadNodePtr& ptr_node_matched);

        void NNSearchDebug(float x, float y, 
            ID& id_data_matched, QuadNodePtr& ptr_node_matched, uint32_t& n_access);
        void cachedNNSearchDebug(float x, float y, QuadNodePtr ptr_node_cached, 
            ID& id_data_matched, QuadNodePtr& ptr_node_matched, uint32_t& n_access);

    // For insert a data
    private:
        InsertData insert_data_;
        inline void resetInsertData();

    // For nearest neighbor search algorithm
    private:
        QuaryData query_data_;
        SimpleStack<QuadNodePtr> simple_stack_;

        inline void resetNNParameters();
        inline void resetQueryData();

    // Related to generate tree.
    private:
        void insertPrivateStack();

        inline void makeChildrenLeaves(QuadNodePtr ptr_parent);

        inline void addDataToNode(QuadNodePtr ptr_node, ElemPtr ptr_elem);
        inline int getNumElemOfNode(QuadNodePtr ptr_node);

        inline void makeBranch(QuadNodePtr ptr_node);

    // Related to NN search (private)
    private:
        void nearestNeighborSearchPrivate();

        inline bool BWBTest( float x, float y, const QuadRect_u& rect, float radius); // Ball Within Bound
        inline bool BOBTest( float x, float y, const QuadRect_u& rect, float radius); // Ball Overlap Bound
        bool findNearestElem(float x, float y, QuadNodePtr ptr_node);

    // Related to cached NN search (private)
    private:
        void cachedNearestNeighborSearchPrivate();

    private:
        QuadParams params_;

    private:
        // Stores all the elements in the quadtree.
        std::vector<ElemPtr>  elements_;
        std::vector<QuadNodePtr>  nodes_;
        
    private:
        // Objectpool for nodes and elements
        QuadNodePtr root_node_;
        std::shared_ptr<ObjectPool<QuadNode>> objpool_node_;
        std::shared_ptr<ObjectPool<Elem>>     objpool_elem_; 

    private:
        uint32_t n_node_activated_;

        float x_normalizer_;
        float y_normalizer_;

    public:
        uint32_t getNumNodesActivated();

    private: 
        // quadtree range (in real scale)
        float x_range_[2]; float y_range_[2];

        uint32_t max_depth_; // Quadtree maximum depth
        uint32_t max_elem_per_leaf_; // The maximum number of elements in the leaf. If maxdepth leaf, no limit to store elements.

    public:     
        void getAllElemRoot();
   
    };

};
#endif
