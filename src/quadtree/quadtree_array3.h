#ifndef _QUADTREE_ARRAY_H_
#define _QUADTREE_ARRAY_H_
#include <cmath>
#include <iostream>
#include <vector>
#include "quadtree/macro_quadtree.h"
#include "simple_stack.h"

using _numeric_uint = uint16_t;

/*
    id_point = point index of the outside
    id_node  = index of the node
    id_elem  = point index in the Quadtree
*/
namespace ArrayBased3 {
using ID = size_t;     // 4 bytes
using Flag = uint8_t;  // 1 bytes

class Quadtree {
 private:
  template <typename T>
  struct Corner {
    T x;
    T y;
    Corner() : x(0), y(0) {}
    Corner(T x_, T y_) : x(x_), y(y_) {}
    Corner(const Corner& pos) : x(pos.x), y(pos.y) {}  // copy constructor
    Corner& operator=(const Corner& c) {
      x = c.x;
      y = c.y;
      return *this;
    }
    Corner& operator+=(const Corner& c) {
      x += c.x;
      y += c.y;
      return *this;
    }
    Corner& operator-=(const Corner& c) {
      x -= c.x;
      y -= c.y;
      return *this;
    }
    friend std::ostream& operator<<(std::ostream& os, const Corner& c) {
      os << "[" << c.x << "," << c.y << "]";
      return os;
    }
  };

  template <typename T>
  struct Rectangle {
    Corner<T> tl;
    Corner<T> br;
    Rectangle() : tl(0, 0), br(0, 0) {}
    Rectangle(const Corner<T>& tl_, const Corner<T>& br_) : tl(tl_), br(br_) {}
    inline Corner<T> getCenter() {
      return Corner<T>((tl.x + br.x) >> 1, (tl.y + br.y) >> 1);
    }
    friend std::ostream& operator<<(std::ostream& os, const Rectangle& c) {
      os << "tl:[" << c.tl.x << "," << c.tl.y << "],br:[" << c.br.x << ","
         << c.br.y << "]";
      return os;
    }
  };

  struct Node {  // 10 bytes (actually 10 bytes)
    // AABB (Axis-alinged Bounding box) is not contained, but just
    // they are just calculated on the fly if needed.
    // This is more efficient because reducing the memory use of a node can
    // proportionally reduce cache misses when you traverse the tree.
    Rectangle<_numeric_uint> rect;  // 2 * 4  = 8 bytes (padding size = 4 bytes)

    // If -2, not initialized (no children.)
    // else if -1,  branch node. (children exist.)
    // else, leaf node.
    uint8_t state;  // 1 byte
    int8_t depth;   // 1 byte

#define STATE_UNACTIVATED 0b0000  // 0
#define STATE_ACTIVATED 0b0001    // 1 (0b0001)
#define STATE_BRANCH 0b0011       // 2 (0b0011)
#define STATE_LEAF 0b0101         // 4 (0b0101)

    Node() : state(STATE_UNACTIVATED), depth(-1){};
    friend std::ostream& operator<<(std::ostream& os, const Node& c) {
      os << "count:[" << c.state << "]";
      return os;
    };

    inline bool isUnactivated() const { return this->state == 0b0000; }
    inline bool isActivated() const { return this->state & STATE_ACTIVATED; }
    inline bool isBranch() const { return this->state == STATE_BRANCH; }
    inline bool isLeaf() const { return this->state == STATE_LEAF; }

    inline void makeUnactivated() { this->state = STATE_UNACTIVATED; }
    inline void makeActivated() { this->state = STATE_ACTIVATED; }
    inline void makeBranch() { this->state = STATE_BRANCH; }
    inline void makeLeaf() { this->state = STATE_LEAF; }
  };

  struct InputData {  // 12 bytes (actually 16 bytes)
    float x_nom;      // 4 bytes
    float y_nom;      // 4 bytes
    ID id_point;      // 4 bytes

    InputData() : id_point(0), x_nom(-1.0f), y_nom(-1.0f){};
    InputData(float x_nom_, float y_nom_, int id_point_)
        : id_point(id_point_), x_nom(x_nom_), y_nom(y_nom_){};
  };
  struct NodeBin {  // 8 bytes
    // Stores the ID for the element (can be used to refer to external data).
    std::vector<ID> elem_ids;  // 8 bytes

    NodeBin() { elem_ids.resize(0); };
    inline void reset() { elem_ids.resize(0); };
    inline int getNumElem() const { return elem_ids.size(); };
  };
  struct QueryData {
    float x;      // 4 bytes
    float y;      // 4 bytes
    float x_nom;  // 4 bytes
    float y_nom;  // 4 bytes

    ID id_point_matched;  // 4 bytes

    ID id_elem_matched;  // 4 bytes
    ID id_node_matched;  // 4 bytes
    ID id_node_cached;   // 4 bytes

    float min_dist2_;
    float min_dist_;
  };

  struct QuadParams {
    float approx_rate;             //  0.3~1.0;
    uint8_t flag_adj_search_only;  // Searching parameter
  };

 public:
  Quadtree(float x_min, float x_max, float y_min, float y_max, size_t max_depth,
           size_t max_elem_per_leaf, float approx_rate = 1.0,
           uint8_t flag_adj = false);
  ~Quadtree();

  void insert(float x, float y, int id_point);
  void NNSearch(float x, float y, ID& id_point_matched, ID& id_node_matched);
  void cachedNNSearch(float x, float y, int id_node_cached,
                      ID& id_point_matched, ID& id_node_matched);

  void NNSearchDebug(float x, float y, ID& id_point_matched,
                     ID& id_node_matched, size_t& n_access);
  void cachedNNSearchDebug(float x, float y, int id_node_cached,
                           ID& id_point_matched, ID& id_node_matched,
                           size_t& n_access);
  const size_t getNumNodesTotal() const;
  const size_t getNumNodesActivated() const;

 private:  // Related to generate tree.
  void insertPrivateStack(const float x_nom, const float y_nom,
                          const ID id_elem);

  inline void makeChildrenLeaves(ID id_child,
                                 const Rectangle<_numeric_uint>& rect);

  inline void addDataToNode(ID id_node, ID id_elem);
  inline int getNumElemOfNode(ID id_node);

  inline void makeBranch(ID id_node);

 private:                               // Related to NN search (private)
  void nearestNeighborSearchPrivate();  // return id_point matched.

  inline bool BWBTest(float x, float y, const Rectangle<_numeric_uint>& rect,
                      float radius);  // Ball Within Bound
  inline bool BOBTest(float x, float y, const Rectangle<_numeric_uint>& rect,
                      float radius);  // Ball Overlap Bound
  bool findNearestElem(float x, float y, const NodeBin& elems);

 private:  // Related to cached NN search (private)
  void cachedNearestNeighborSearchPrivate();

 private:
  QuadParams params_;

 private:
  // Stores all the elements in the quadtree.
  std::vector<InputData> input_points_;

 private:
  // Stores all the nodes in the quadtree. The second node in this
  // sequence is always the root.
  // index 0 is not used.
  std::vector<Node> nodes_;
  std::vector<NodeBin> nodes_bin_;
  size_t n_nodes_total_;
  size_t n_nodes_activated_;
  // |  1  |  2  |  3  |  4  |  5  |  ...
  // | root| tl0 | bl0 | tr0 | br0 |  ...
  // Z-order
  //  node_index * 4  - 2 -> first child id (child 0~3) ( (node_index << 2) - 2
  //  )
  // (node_index + 2) / 4 -> parent id ( (node_index + 2) >> 2 )
  // (node_index + 2) % 4 -> quadrant number of this node
  /*
  depth 0 (root)
      1
  depth 1
      2 3
      4 5
  depth 2
       6  7 10 11
       8  9 12 13
      14 15 18 19
      16 17 20 21
  */
 private:  // quadtree range (in real scale)
  float x_range_[2];
  float y_range_[2];
  float normalizer_;

  size_t max_depth_;  // Quadtree maximum depth
  size_t
      max_elements_per_leaf_;  // The maximum number of elements in the leaf. If
                               // maxdepth leaf, no limit to store elements.

 private:  // For nearest neighbor search algorithm
  QueryData query_data_;
  SimpleStack<ID> simple_stack_;

  inline void resetNNParameters();
  inline void resetQueryData();
};
};  // namespace ArrayBased3
#endif