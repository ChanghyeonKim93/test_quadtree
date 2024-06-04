#ifndef _QUADTREE_POINTER_REFACTOR_H_
#define _QUADTREE_POINTER_REFACTOR_H_
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include "quadtree/macro_quadtree.h"
#include "quadtree/object_pool.h"
#include "quadtree/simple_stack.h"

namespace quadtree {
using ID = int;        // 4 bytes
using Flag = uint8_t;  // 1 bytes
template <typename T>
struct Pos2  // sizeof(T)*2 bytes
{
  T x;
  T y;
  Pos2() : x(0), y(0) {}
  Pos2(T input_x, T input_y) : x(input_x), y(input_y) {}
  Pos2(const Pos2& pos) : x(pos.x), y(pos.y) {}  // copy constructor
  Pos2& operator=(const Pos2& pos) {             // copy insert constructor
    x = pos.x;
    y = pos.y;
    return *this;
  }
  Pos2& operator+=(const Pos2& pos) {
    x += pos.x;
    y += pos.y;
    return *this;
  }
  Pos2& operator-=(const Pos2& pos) {
    x -= pos.x;
    y -= pos.y;
    return *this;
  }
  friend std::ostream& operator<<(std::ostream& os, const Pos2& c) {
    os << "[" << c.x << "," << c.y << "]";
    return os;
  }
};

template <typename T>
struct Rect  // sizeof(T)*4 bytes
{
  Pos2<T> tl;
  Pos2<T> br;
  Rect() : tl(0, 0), br(0, 0) {}
  Rect(const Pos2<T>& input_tl, const Pos2<T>& input_br)
      : tl(input_tl), br(input_br) {}
  inline Pos2<T> getCenter() {
    return Pos2<T>((tl.x + br.x) * 0.5, (tl.y + br.y) * 0.5);
  }
  friend std::ostream& operator<<(std::ostream& os, const Rect& c) {
    os << "tl:[" << c.tl.x << "," << c.tl.y << "],br:[" << c.br.x << ","
       << c.br.y << "]";
    return os;
  }
};

using QuadUint = uint16_t;
using Rect_u = Rect<QuadUint>;
using Rect_f = Rect<float>;

struct LinkedElement;
using LinkedElementPtr = LinkedElement*;
struct LinkedElement {  // 20 bytes (actually 16 bytes)
  LinkedElementPtr next;
  float normalized_x;  // 4 bytes
  float normalized_y;  // 4 bytes
  ID data_id;          // 4 bytes, (external index)

  LinkedElement()
      : next(nullptr), data_id(0), normalized_x(-1.0f), normalized_y(-1.0f) {}
  LinkedElement(const float input_normalized_x, const float input_normalized_y,
                const int input_data_id)
      : next(nullptr),
        data_id(input_data_id),
        normalized_x(input_normalized_x),
        normalized_y(input_normalized_y) {}
  inline void ResetAndSetData(const float input_normalized_x,
                              const float input_normalized_y,
                              const int input_data_id) {
    next = nullptr;
    normalized_x = input_normalized_x;
    normalized_y = input_normalized_y;
    data_id = input_data_id;
  }
};

struct Node;
using NodePtr = Node*;
struct Node {                     // 46 bytes
  NodePtr parent;                 // 8
  NodePtr first_child;            // 8
  Rect_u rect;                    // 2 * 4 (8)
  LinkedElementPtr element_head;  // 8 bytes
  LinkedElementPtr element_tail;  // 8 bytes
  uint8_t state;                  // 1
  int8_t depth;                   // 1
  uint32_t n_elem;                // 4 byte

#define STATE_UNACTIVATED 0b0000  // 0
#define STATE_ACTIVATED 0b0001    // 1 (0b0001)
#define STATE_BRANCH 0b0011       // 3 (0b0011)
#define STATE_LEAF 0b0101         // 5 (0b0101)
  inline void MakeThisActivated() { state = STATE_ACTIVATED; }
  inline void MakeThisUnactivated() {
    state = STATE_UNACTIVATED;
    n_elem = 0;
  }
  inline void MakeThisBranch() {
    state = STATE_BRANCH;
    n_elem = 0;
  }
  inline void MakeThisLeaf() { state = STATE_LEAF; }
  inline const bool IsActivated() const { return state & 0b0001; }
  inline const bool IsBranch() const { return state == 0b0011; }
  inline const bool IsLeaf() const { return state == 0b0101; }

  Node()
      : state(STATE_UNACTIVATED),
        element_head(nullptr),
        element_tail(nullptr),
        depth(-1),
        parent(nullptr),
        first_child(nullptr),
        n_elem(0) {}
  friend std::ostream& operator<<(std::ostream& os, const Node& c) {
    os << "count:[" << c.state << "]";
    return os;
  }
  inline void Reset() {
    element_head = nullptr;
    element_tail = nullptr;
    rect.tl.x = 0;
    rect.tl.y = 0;
    rect.br.x = 0;
    rect.br.y = 0;
    parent = nullptr;
    first_child = nullptr;
    state = STATE_UNACTIVATED;
    depth = -1;
    n_elem = 0;
  }
};

class Quadtree {
 private:
  struct InsertData {              // 16 bytes (actually 16 bytes)
    float normalized_x;            // 4 bytes
    float normalized_y;            // 4 bytes
    LinkedElementPtr element_ptr;  // 8 bytes

    InsertData()
        : normalized_x(-1.0f), normalized_y(-1.0f), element_ptr(nullptr){};
    void ResetAndSetData(const float x_nom_in, const float y_nom_in,
                         LinkedElementPtr elem_) {
      normalized_x = x_nom_in;
      normalized_y = y_nom_in;
      element_ptr = elem_;
    }
  };

  struct QueryData {
    float x;             // 4 bytes
    float y;             // 4 bytes
    float normalized_x;  // 4 bytes
    float normalized_y;  // 4 bytes

    ID matched_data_id;        // 4 bytes
    NodePtr matched_node_ptr;  // 4 bytes
    NodePtr cached_node_ptr;   // 4 bytes

    float min_distance_sqaured_;
    float min_distance_;
  };

  struct Parameters {
    // Distance parameter
    float distance_approximate_rate;  // 0.3~1.0;

    // Searching parameter
    uint8_t flag_adjacent_search_only;
  };

 public:
  Quadtree(const float x_min, const float x_max, const float y_min,
           const float y_max, uint32_t max_depth, uint32_t max_elem_per_leaf,
           float approx_rate = 1.0, uint8_t flag_adj = false);
  ~Quadtree();

  void InsertReferenceData(const float x, const float y, const int data_id);

  void SearchNearestNeighbor(const float x, const float y, ID& matched_data_id,
                             NodePtr& matched_node_ptr);
  void SearchNearestNeighborWithNodeCache(const float x, const float y,
                                          const NodePtr cached_node_ptr,
                                          ID& matched_data_id,
                                          NodePtr& matched_node_ptr);

  void SearchNearestNeighbor_debug(const float x, const float y,
                                   ID& matched_data_id,
                                   NodePtr& matched_node_ptr,
                                   uint32_t& n_access);
  void SearchNearestNeighborWithNodeCache_debug(const float x, const float y,
                                                const NodePtr cached_node_ptr,
                                                ID& matched_data_id,
                                                NodePtr& matched_node_ptr,
                                                uint32_t& n_access);

  uint32_t GetNumActivatedNodes();
  void GetAllElementInRoot();

 private:  // For insert a data
  InsertData insert_data_;

 private:  // For nearest neighbor search algorithm
  QueryData query_data_;
  SimpleStack<NodePtr> simple_stack_;

  inline void ResetNNParameters();
  inline void ResetQueryData();

 private:  // Related to generate tree.
  void InsertPrivateStack();

  inline void MakeChildrenAsLeaf(NodePtr ptr_parent);

  inline void AddDataToNode(NodePtr node_ptr, LinkedElementPtr element_ptr);
  inline int GetNumElementInNode(NodePtr node_ptr);

  inline void MakeThisAsBranch(NodePtr node_ptr);

 private:  // Related to NN search (private)
  void SearchNearestNeighborPrivate();

  inline bool CheckBallWithinBound(const float x, const float y,
                                   const Rect_u& rect,
                                   const float radius);  // Ball Within Bound
  inline bool CheckBallOverlapBound(const float x, const float y,
                                    const Rect_u& rect,
                                    const float radius);  // Ball Overlap Bound
  bool FindNearestElement(const float x, const float y, NodePtr node_ptr);

 private:  // Related to cached NN search (private)
  void SearchNearestNeighborWithNodeCachePrivate();

 private:
  Parameters parameters_;

 private:  // Stores all the elements in the quadtree.
  std::vector<LinkedElementPtr> element_ptr_list_;
  std::vector<NodePtr> node_ptr_list_;

 private:  // Objectpool for nodes and elements
  NodePtr root_node_;

 private:
  uint32_t n_node_activated_;

  float x_normalizer_;
  float y_normalizer_;

 private:  // quadtree range (in real scale)
  float x_range_[2];
  float y_range_[2];

  uint32_t max_depth_;          // Quadtree maximum depth
  uint32_t max_elem_per_leaf_;  // The maximum number of elements in the leaf.
                                // If maxdepth leaf, no limit to store elements.

 public:
  static std::unique_ptr<ObjectPool<Node>> objpool_node_;
  static std::unique_ptr<ObjectPool<LinkedElement>> objpool_elem_;
};
};  // namespace quadtree
#endif
