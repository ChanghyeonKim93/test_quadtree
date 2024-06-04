#ifndef OCTREE_POINTER_BASE_H_
#define OCTREE_POINTER_BASE_H_

#include <iostream>

#include "Eigen/Dense"

namespace octree {

struct Parameters {
  struct {
    struct {
      double min;
      double max;
    } x;
    struct {
      double min;
      double max;
    } y;
    struct {
      double min;
      double max;
    } z;
  } range;
  int max_tree_depth{7};
  int max_elements_per_leaf{20};
  double distance_approximation_rate{1.0};
};

struct OctreeNode;
using OctreeNodePtr = OctreeNode*;
struct OctreeNode {
  OctreeNodePtr parent;
  OctreeNodePtr first_child;
};

class Octree {
 public:
  Octree(const Parameters parameters);

  void InsertData(const Eigen::Vector3d& point, const uint64_t id);
  void SearchNearestNeighbor(const Eigen::Vector3d& point, const int* node_id);
  void SearchNearestNeighborWithCachedNode();

 private:
  bool CheckBallWithinBound(const Eigen::Vector3d& center_point,
                            const double radius);
  bool CheckBallOverlapBound(const Eigen::Vector3d& center_point,
                             const double radius);

 private:
  double x_normalizer_;
  double y_normalizer_;
  double z_normalizer_;
};

}  // namespace octree

#endif  // OCTREE_POINTER_BASE_H_