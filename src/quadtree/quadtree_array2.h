#ifndef _QUADTREE_ARRAY_H_
#define _QUADTREE_ARRAY_H_
#include <cmath>
#include <iostream>
#include <vector>
#include "quadtree/macro_quadtree.h"
#include "simple_stack.h"

namespace ArrayBased2 {
using ID = uint32_t;   // 4 bytes
using Flag = uint8_t;  // 1 bytes

class Quadtree {
 private:
  template <typename T>
  struct Pos2d {  // sizeof(T)*2 bytes
    T x;
    T y;

    Pos2d() : x(0), y(0){};
    Pos2d(T x_, T y_) : x(x_), y(y_){};
    Pos2d(const Pos2d& pos) : x(pos.x), y(pos.y){};  // copy constructor

    Pos2d& operator=(const Pos2d& c) {  // copy insert constructor
      x = c.x;
      y = c.y;
      return *this;
    };
    Pos2d& operator+=(const Pos2d& c) {
      x += c.x;
      y += c.y;
      return *this;
    };
    Pos2d& operator-=(const Pos2d& c) {
      x -= c.x;
      y -= c.y;
      return *this;
    };
    friend std::ostream& operator<<(std::ostream& os, const Pos2d& c) {
      os << "[" << c.x << "," << c.y << "]";
      return os;
    };
  };

  template <typename T>
  struct QuadRect {  // sizeof(T)*4 bytes
    Pos2d<T> tl;
    Pos2d<T> br;
    QuadRect() : tl(0, 0), br(0, 0){};
    QuadRect(const Pos2d<T>& tl_, const Pos2d<T>& br_) : tl(tl_), br(br_){};

    inline Pos2d<T> getCenter() {
      return Pos2d<T>((double)(tl.x + br.x) * 0.5, (double)(tl.y + br.y) * 0.5);
    };

    friend std::ostream& operator<<(std::ostream& os, const QuadRect& c) {
      os << "tl:[" << c.tl.x << "," << c.tl.y << "],br:[" << c.br.x << ","
         << c.br.y << "]";
      return os;
    };
  };

  using QuadUint = uint16_t;
  using QuadRect_u = QuadRect<QuadUint>;
  using QuadRect_f = QuadRect<float>;

  struct Elem {  // 20 bytes (actually 16 bytes)
    Elem* next;
    float x_nom;  // 4 bytes
    float y_nom;  // 4 bytes
    ID id_data;   // 4 bytes, (external index)

    Elem() : next(nullptr), id_data(0), x_nom(-1.0f), y_nom(-1.0f){};
    Elem(float x_nom_, float y_nom_, int id_data_)
        : next(nullptr), id_data(id_data_), x_nom(x_nom_), y_nom(y_nom_){};
  };

  struct QuadNode {  // 19 bytes (actually 10 bytes)
    // AABB (Axis-alinged Bounding box) is not contained, but just
    // they are just calculated on the fly if needed.
    // This is more efficient because reducing the memory use of a node can
    // proportionally reduce cache misses when you traverse the tree.
    QuadRect_u rect;  // 2 * 4  = 8 bytes (padding size = 4 bytes)
    Elem* elem;       // 8 bytes

    // If -2, not initialized (no children.)
    // else if -1,  branch node. (children exist.)
    // else, leaf node.
    uint8_t state;   // 1 byte (-2 : unactivated, -1: branch, 0: leaf)
    int8_t depth;    // 1 byte
    uint8_t n_elem;  // 1 byte

#define STATE_UNACTIVATED 0b0000  // 0
#define STATE_ACTIVATED 0b0001    // 1 (0b0001)
#define STATE_BRANCH 0b0011       // 2 (0b0011)
#define STATE_LEAF 0b0101         // 4 (0b0101)

#define IS_UNACTIVATED(nd) ((nd).state == 0b0000)
#define IS_ACTIVATED(nd) ((nd).state & STATE_ACTIVATED)
#define IS_BRANCH(nd) ((nd).state == STATE_BRANCH)
#define IS_LEAF(nd) ((nd).state == STATE_LEAF)

#define MAKE_UNACTIVATE(nd)         \
  {                                 \
    (nd).state = STATE_UNACTIVATED; \
    (nd).n_elem = 0;                \
  }
#define MAKE_ACTIVATE(nd) ((nd).state = STATE_ACTIVATED)
#define MAKE_BRANCH(nd)        \
  {                            \
    (nd).state = STATE_BRANCH; \
    (nd).n_elem = 0;           \
  }
#define MAKE_LEAF(nd) ((nd).state = STATE_LEAF)

    QuadNode()
        : state(STATE_UNACTIVATED), depth(-1), elem(nullptr), n_elem(0){};
    friend std::ostream& operator<<(std::ostream& os, const QuadNode& c) {
      os << "count:[" << c.state << "]";
      return os;
    };
  };

  struct InsertData {  // 12 bytes (actually 16 bytes)
    float x_nom;       // 4 bytes
    float y_nom;       // 4 bytes
    Elem* elem;        // current element pointer

    InsertData() : x_nom(-1.0f), y_nom(-1.0f), elem(nullptr){};
    void setData(float x_nom_, float y_nom_, Elem* elem_) {
      x_nom = x_nom_;
      y_nom = y_nom_;
      elem = elem_;
    };
  };

  struct QueryData {
    float x;      // 4 bytes
    float y;      // 4 bytes
    float x_nom;  // 4 bytes
    float y_nom;  // 4 bytes

    ID id_node_cached;  // 4 bytes

    ID id_data_matched;  // 4 bytes
    ID id_node_matched;  // 4 bytes

    float min_dist2_;
    float min_dist_;
  };

  struct QuadParams {
    // Distance parameter
    float approx_rate;  // 0.3~1.0;

    // Searching parameter
    uint8_t flag_adj_search_only;
  };

 public:
  Quadtree(float x_min, float x_max, float y_min, float y_max,
           uint32_t max_depth, uint32_t max_elem_per_leaf,
           float approx_rate = 1.0, uint8_t flag_adj = false);
  ~Quadtree();

  void insert(float x, float y, int id_data);
  void NNSearch(float x, float y, ID& id_data_matched, ID& id_node_matched);
  void cachedNNSearch(float x, float y, int id_node_cached, ID& id_data_matched,
                      ID& id_node_matched);

  void NNSearchDebug(float x, float y, ID& id_data_matched, ID& id_node_matched,
                     uint32_t& n_access);
  void cachedNNSearchDebug(float x, float y, int id_node_cached,
                           ID& id_data_matched, ID& id_node_matched,
                           uint32_t& n_access);

  // For insert a data
 private:
  InsertData insert_data_;
  inline void resetInsertData();

  // For nearest neighbor search algorithm
 private:
  QueryData query_data_;
  SimpleStack<ID> simple_stack_;

  inline void resetNNParameters();
  inline void resetQueryData();

  // Related to generate tree.
 private:
  void insertPrivate(ID id_node, uint8_t depth);
  void insertPrivateStack();

  inline void makeChildrenLeaves(ID id_child, const QuadRect_u& rect);

  inline void addDataToNode(ID id_node, Elem* elem);
  inline int getNumElemOfNode(ID id_node);

  inline void makeBranch(ID id_node);

  // Related to NN search (private)
 private:
  void nearestNeighborSearchPrivate();  // return id_data matched.

  inline bool BWBTest(float x, float y, const QuadRect_u& rect,
                      float radius);  // Ball Within Bound
  inline bool BOBTest(float x, float y, const QuadRect_u& rect,
                      float radius);  // Ball Overlap Bound
  bool findNearestElem(float x, float y, const ID& id_node);

  // Related to cached NN search (private)
 private:
  void cachedNearestNeighborSearchPrivate();

 private:
  QuadParams params_;

 private:
  // Stores all the elements in the quadtree.
  std::vector<Elem*> elements_;

 private:
  // Stores all the nodes in the quadtree. The second node in this
  // sequence is always the root.
  // index 0 is not used.
  std::vector<QuadNode> nodes;
  uint32_t n_nodes_;
  uint32_t n_node_activated_;
  // |  1  |  2  |  3  |  4  |  5  |  ...
  // | root| tl0 | bl0 | tr0 | br0 |  ...
  // Z-order
  //  id_node * 4  - 2 -> first child id (child 0~3)
  // (id_node + 2) / 4 -> parent id
  // (id_node + 2) % 4 -> quadrant index of this node

  float x_normalizer_;
  float y_normalizer_;

 public:
  uint32_t getNumNodes();
  uint32_t getNumNodesActivated();

 private:
  // quadtree range (in real scale)
  float x_range_[2];
  float y_range_[2];

  uint32_t max_depth_;          // Quadtree maximum depth
  uint32_t max_elem_per_leaf_;  // The maximum number of elements in the leaf.
                                // If maxdepth leaf, no limit to store elements.
};
};  // namespace ArrayBased2
#endif