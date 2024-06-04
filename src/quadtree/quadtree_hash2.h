#ifndef _QUADTREE_HASH2_H_
#define _QUADTREE_HASH2_H_
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>
#include "simple_stack.h"

#include "quadtree/macro_quadtree.h"

using _numeric_uint = uint16_t;

/*
  bit 1-4: depth (0~16)
  bit 5  : leaf node
  bit 6  : branch node
  bit 7  : reserved
  bit 8  : reserved
*/
/*
num &= ~( 1 << n ) make n-th bit 0

num |= (1 << n) make n-th bit 1

*/

/*
x번째 비트를 1로 SET
OR (|)를 사용.
value |= << x;


x번째 비트를 0으로 SET
AND (&)를 사용.
value &= ~(1<<x);


x번째 비트가 1이면 0, 0이면 1로 SET
XOR (^)를 사용.
value ^= 1 << x;


x번째 비트가 1이면 1을, 0이면 0을 리턴
AND (&)를 사용.
(value >> x ) & 1;
*/

/*
  bit 1-4: depth (0~16)
  bit 5  : leaf node
  bit 6  : branch node
  bit 7  : reserved
  bit 8  : reserved
*/

#define BIT_LEAF 0b00010000
#define BIT_BRANCH 0b00100000
#define BITS_LEAF_BRANCH 0b00110000

#define RESET_STATE(state) ((state) == 0b00000000)
#define SET_DEPTH(state, d)               \
  {                                       \
    (state) &= ~0b00001111;               \
    (state) |= static_cast<uint8_t>((d)); \
  }
#define GET_DEPTH(state) \
  (static_cast<uint16_t>((state) & 0b00001111))  // lower four bytes
#define PLUS_DEPTH(state) ((state) += 1)
#define MINUS_DEPTH(state) ((state) -= 1)

#define MAKE_UNACTIVATED(state) \
  {                             \
    (state) &= ~BIT_BRANCH;     \
    (state) &= ~BIT_LEAF;       \
  }
#define MAKE_LEAF(state)    \
  {                         \
    (state) &= ~BIT_BRANCH; \
    (state) |= BIT_LEAF;    \
  }
#define MAKE_BRANCH(state) \
  {                        \
    (state) &= ~BIT_LEAF;  \
    (state) |= BIT_BRANCH; \
  }

#define IS_ACTIVATED(state) (((state) & BITS_LEAF_BRANCH) > 0)
#define IS_BRANCH(state) (((state) & BIT_BRANCH) > 0)
#define IS_LEAF(state) (((state) & BIT_LEAF) > 0)

#define CHECK_N_TH_BIT(state, n) (((state) & (1 << (n))) > 0)

namespace HashBased2 {
using ID = uint32_t;
using Flag = uint8_t;

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

  class Node {
   private:
    Rectangle<uint16_t> rect;
    uint8_t state;

   public:
    Node() : state(0b00000000) {}
    inline bool isUnactivated() const { return this->state == 0b0000; }
    inline bool isActivated() const { return IS_ACTIVATED(this->state); }
    inline bool isBranch() const { return IS_BRANCH(this->state); }
    inline bool isLeaf() const { return IS_LEAF(this->state); }

    inline void makeUnactivated() { MAKE_UNACTIVATED(this->state); }
    inline void makeBranch() { MAKE_BRANCH(this->state); }
    inline void makeLeaf() { MAKE_LEAF(this->state); }
    inline void setDepth(const uint8_t d) { SET_DEPTH(this->state, d); }
    // inline void increaseDepth() { PLUS_DEPTH(this->state); }
    // inline void decreaseDepth() { MINUS_DEPTH(this->state); }
    inline uint8_t getDepth() const { return GET_DEPTH(this->state); }
    inline void setRect(const Rectangle<uint16_t>& rect_in) {
      rect.tl.x = rect_in.tl.x;
      rect.tl.y = rect_in.tl.y;
      rect.br.x = rect_in.br.x;
      rect.br.y = rect_in.br.y;
    }
    const Rectangle<uint16_t>& getRect() const { return rect; }
  };

  struct InputData {  // 12 bytes (actually 16 bytes)
    float x_nom;      // 4 bytes
    float y_nom;      // 4 bytes
    ID id_point;      // 4 bytes

    InputData() : id_point(0), x_nom(-1.0f), y_nom(-1.0f){};
    InputData(float x_nom_, float y_nom_, int id_point_)
        : id_point(id_point_), x_nom(x_nom_), y_nom(y_nom_){};
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

  const size_t getNumNodesActivated() const;

 private:  // Related to generate tree.
  void insertPrivateStack(const float x_nom, const float y_nom,
                          const ID id_elem);
  void addNewNode(const uint32_t node_id, const uint8_t depth,
                  const Rectangle<uint16_t>& rect);
  Rectangle<uint16_t> getSubRectangle(const Rectangle<uint16_t>& rect,
                                      const bool flag_sn, const bool flag_ew);

  inline void addDataToNode(ID node_id, ID elem_id);
  inline size_t getNumElemOfNode(ID node_id);

  inline void makeBranch(ID node_id);

 private:
  QuadParams params_;

 private:
  // Stores all the elements in the quadtree.
  std::vector<InputData> input_points_;

 private:
  std::unordered_map<uint32_t, Node> node_list_;
  std::unordered_map<uint32_t, std::vector<uint32_t>> node_elem_id_list_;
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
};  // namespace HashBased2
#endif