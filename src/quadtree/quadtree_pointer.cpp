#include "quadtree/quadtree_pointer.h"

namespace PointerBased{
    Quadtree::Quadtree(
        float x_min, float x_max,
        float y_min, float y_max,
        uint32_t max_depth, uint32_t max_elem_per_leaf,
        float approx_rate, uint8_t flag_adj)
    : max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max},
    n_node_activated_(1), max_elem_per_leaf_(max_elem_per_leaf)
    {
        // Make root node.
        QuadNodePtr root_node;
    };
};