#include "quadtree/quadtree_pointer_refactor.h"

namespace quadtree
{
  Quadtree::Quadtree(
      const float x_min, const float x_max,
      const float y_min, const float y_max,
      std::shared_ptr<ObjectPool<Node>> objpool_node, std::shared_ptr<ObjectPool<LinkedElement>> objpool_elem,
      uint32_t max_depth, uint32_t max_elem_per_leaf,
      const float distance_approximate_rate, uint8_t flag_adj)
      : max_depth_(max_depth), x_range_{x_min, x_max}, y_range_{y_min, y_max},
        n_node_activated_(1), max_elem_per_leaf_(max_elem_per_leaf)
  {
    std::cout << "sizeof(QuadNode): " << sizeof(Node) << "Bytes" << std::endl;

    objpool_node_ = objpool_node;
    objpool_elem_ = objpool_elem;

    // Initialize a root node.
    root_node_ = new Node();
    root_node_->depth = 0;
    MAKE_LEAF_P(root_node_);
    // nodes_.push_back(root_node_);

    // Root size.
    QuadUint br_x = 1 << 15;
    QuadUint br_y = 1 << 15;

    root_node_->rect.tl = Pos2<QuadUint>(0, 0);
    root_node_->rect.br = Pos2<QuadUint>(br_x, br_y);

    // Calculate normalizers
    x_normalizer_ = static_cast<float>(root_node_->rect.br.x) / (x_range_[1] - x_range_[0]);
    y_normalizer_ = static_cast<float>(root_node_->rect.br.y) / (y_range_[1] - y_range_[0]);

    if (x_normalizer_ > y_normalizer_)
      y_normalizer_ = x_normalizer_;
    else
      x_normalizer_ = y_normalizer_;

    // Parameter setting
    parameters_.distance_approximate_rate = distance_approximate_rate;
    if (parameters_.distance_approximate_rate > 1.0f)
      parameters_.distance_approximate_rate = 1.0f;
    if (parameters_.distance_approximate_rate < 0.5f)
      parameters_.distance_approximate_rate = 0.5f;
    parameters_.distance_approximate_rate = parameters_.distance_approximate_rate * parameters_.distance_approximate_rate;

    parameters_.flag_adjacent_search_only = flag_adj;
  }

  Quadtree::~Quadtree()
  {
    delete root_node_;
    for (auto const &node_ptr : node_ptr_list_)
    {
      node_ptr->Reset();
      objpool_node_->ReturnObject(node_ptr);
    }
    for (auto const &element_ptr : element_ptr_list_)
    {
      objpool_elem_->ReturnObject(element_ptr);
    }
    std::cout << "Quadtree is deleted.\n";
  }

  void Quadtree::ResetNNParameters()
  {
    // Initialize values
    simple_stack_.clear();
    simple_stack_.clearTotalAccess();
    query_data_.min_distance_sqaured_ = std::numeric_limits<float>::max();
    query_data_.min_distance_ = std::numeric_limits<float>::max();
  }

  inline void Quadtree::ResetQueryData()
  {
    query_data_.matched_data_id = 0;
    query_data_.cached_node_ptr = root_node_;
    query_data_.matched_node_ptr = root_node_;
    query_data_.min_distance_sqaured_ = std::numeric_limits<float>::max();
    query_data_.min_distance_ = std::numeric_limits<float>::max();
    query_data_.x = -1.0f;
    query_data_.y = -1.0f;
    query_data_.normalized_x = -1.0f;
    query_data_.normalized_y = -1.0f;
  }

  void Quadtree::InsertReferenceData(
      const float x, const float y, const int data_id)
  {
    if (x < x_range_[0] || x > x_range_[1])
      return;
    if (y < y_range_[0] || y > y_range_[1])
      return;

    // normalize input point coordinates
    const auto normalized_x = x * x_normalizer_;
    const auto normalized_y = y * y_normalizer_;

    // Add the input data as an Element object
    LinkedElementPtr new_element_ptr = objpool_elem_->GetObject();
    new_element_ptr->ResetAndSetData(normalized_x, normalized_y, data_id);

    element_ptr_list_.push_back(new_element_ptr);

    // Initialize the 'insert_data_'
    // This is to avoid recursive copies of recursive call of function inputs)
    insert_data_.ResetAndSetData(normalized_x, normalized_y, new_element_ptr);

    // Insert query_data_ into the quadtree.
    // this->insertPrivate(1, 0); // Recursion-based approach (DFS)
    this->InsertPrivateStack(); // Stack-based approach (DFS), faster than Recursion-based one 1.2 times.
  }

  void Quadtree::InsertPrivateStack()
  {
    float &x_nom = insert_data_.normalized_x;
    float &y_nom = insert_data_.normalized_y;

    NodePtr node_ptr = root_node_;
    node_ptr->depth = 0;

    while (true)
    {
      const auto &depth = node_ptr->depth;

      if (IS_BRANCH_P(node_ptr))
      {
        // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
        Flag flag_sn, flag_ew;
        FIND_QUADRANT(x_nom, y_nom, node_ptr->rect, flag_sn, flag_ew);
        uint8_t child_index = GET_CHILD_INDEX(flag_sn, flag_ew);

        node_ptr = node_ptr->first_child + child_index;
        node_ptr->depth = depth + 1;
      }
      else if (IS_LEAF_P(node_ptr))
      {
        AddDataToNode(node_ptr, insert_data_.element_ptr);

        if (depth < max_depth_)
        {
          // make all child to leaf (with no element)
          const auto n_elem = GetNumElementInNode(node_ptr);
          if (n_elem > max_elem_per_leaf_)
          {
            MakeChildrenLeaves(node_ptr);
            n_node_activated_ += 4;

            // Do divide
            LinkedElementPtr elem_tmp = node_ptr->element_head;
            while (elem_tmp != nullptr)
            {
              LinkedElementPtr elem_next = elem_tmp->next;

              elem_tmp->next = nullptr;

              Flag flag_sn, flag_ew;
              FIND_QUADRANT(elem_tmp->normalized_x, elem_tmp->normalized_y, node_ptr->rect, flag_sn, flag_ew);
              uint8_t child_index = GET_CHILD_INDEX(flag_sn, flag_ew);

              AddDataToNode(node_ptr->first_child + child_index, elem_tmp);
              elem_tmp = elem_next;
            }

            // make this node as a branch
            MakeThisAsBranch(node_ptr);
          }
        }
        break;
      }
    }
  }

  void Quadtree::SearchNearestNeighbor(
      const float x, const float y,
      ID &matched_data_id, NodePtr &matched_node_ptr)
  {
    query_data_.x = x;
    query_data_.y = y;
    query_data_.normalized_x = x * x_normalizer_;
    query_data_.normalized_y = y * y_normalizer_;
    // query_data_.cached_node_ptr = root_node_; // non-cached case now.

    SearchNearestNeighborPrivate();

    matched_data_id = query_data_.matched_data_id;
    matched_node_ptr = query_data_.matched_node_ptr;
  }

  void Quadtree::SearchNearestNeighbor_debug(
      const float x, const float y,
      ID &matched_data_id, NodePtr &matched_node_ptr, uint32_t &num_node_access)
  {
    SearchNearestNeighbor(x, y, matched_data_id, matched_node_ptr);
    num_node_access = simple_stack_.getTotalAccess();
  }

  void Quadtree::SearchNearestNeighborWithNodeCache_debug(
      const float x, const float y, NodePtr cached_node_ptr,
      ID &matched_data_id, NodePtr &matched_node_ptr, uint32_t &num_node_access)
  {
    SearchNearestNeighborWithNodeCache(x, y, cached_node_ptr, matched_data_id, matched_node_ptr);
    num_node_access = simple_stack_.getTotalAccess();
  }

  inline void Quadtree::AddDataToNode(NodePtr node_ptr, LinkedElementPtr element_ptr)
  {
    element_ptr->next = nullptr;
    if (node_ptr->element_head != nullptr)
    { // not empty
      node_ptr->element_tail->next = element_ptr;
      node_ptr->element_tail = element_ptr;
    }
    else
    { // empty list
      node_ptr->element_head = element_ptr;
      node_ptr->element_tail = element_ptr;
    }
    ++node_ptr->n_elem;
  }

  bool Quadtree::FindNearestElement(const float x, const float y, NodePtr node_ptr)
  {
    bool is_new_nearest_found = false;
    LinkedElementPtr elem = node_ptr->element_head;
    while (elem != nullptr)
    {
      float current_distance_squared = DIST_EUCLIDEAN(x, y, elem->normalized_x, elem->normalized_y);
      if (current_distance_squared < query_data_.min_distance_sqaured_)
      {
        // std::cout << "min dist chaged: " << query_data_.min_dist2_ <<"," <<dist_temp<<std::endl;
        this->query_data_.matched_data_id = elem->data_id;
        this->query_data_.min_distance_sqaured_ = current_distance_squared * parameters_.distance_approximate_rate;
        this->query_data_.min_distance_ = std::sqrt(query_data_.min_distance_sqaured_);
        is_new_nearest_found = true;
      }
      elem = elem->next;
    }
    return is_new_nearest_found;
  }

  void Quadtree::SearchNearestNeighborWithNodeCache(
      const float x, const float y, NodePtr cached_node_ptr,
      ID &matched_data_id, NodePtr &matched_node_ptr)
  {
    query_data_.x = x;
    query_data_.y = y;
    query_data_.normalized_x = x * x_normalizer_;
    query_data_.normalized_y = y * y_normalizer_;
    query_data_.cached_node_ptr = cached_node_ptr;

    SearchNearestNeighborWithNodeCachePrivate();

    matched_data_id = query_data_.matched_data_id;
    matched_node_ptr = query_data_.matched_node_ptr;
  }

  inline int Quadtree::GetNumElementInNode(NodePtr node_ptr)
  {
    return node_ptr->n_elem;
  }

  inline void Quadtree::MakeChildrenLeaves(NodePtr ptr_parent)
  {
    Rect_u &rect = ptr_parent->rect;
    QuadUint center_x = (rect.tl.x + rect.br.x) >> 1;
    QuadUint center_y = (rect.tl.y + rect.br.y) >> 1;

    // Get four objects consecutive memories
    ptr_parent->first_child = objpool_node_->GetObjectQuadruple();
    node_ptr_list_.push_back(ptr_parent->first_child);
    node_ptr_list_.push_back(ptr_parent->first_child + 1);
    node_ptr_list_.push_back(ptr_parent->first_child + 2);
    node_ptr_list_.push_back(ptr_parent->first_child + 3);

    NodePtr ptr_child = ptr_parent->first_child;

    ptr_child->Reset(); // (0,0) (top left)
    MAKE_LEAF_P(ptr_child);
    ptr_child->rect.tl.x = rect.tl.x;
    ptr_child->rect.tl.y = rect.tl.y;
    ptr_child->rect.br.x = center_x;
    ptr_child->rect.br.y = center_y;
    ptr_child->parent = ptr_parent;
    ptr_child->depth = ptr_parent->depth + 1;

    (++ptr_child)->Reset(); // (0,1) (top right)
    MAKE_LEAF_P(ptr_child);
    ptr_child->rect.tl.x = center_x;
    ptr_child->rect.tl.y = rect.tl.y;
    ptr_child->rect.br.x = rect.br.x;
    ptr_child->rect.br.y = center_y;
    ptr_child->parent = ptr_parent;
    ptr_child->depth = ptr_parent->depth + 1;

    (++ptr_child)->Reset(); // (1,0) (bot left)
    MAKE_LEAF_P(ptr_child);
    ptr_child->rect.tl.x = rect.tl.x;
    ptr_child->rect.tl.y = center_y;
    ptr_child->rect.br.x = center_x;
    ptr_child->rect.br.y = rect.br.y;
    ptr_child->parent = ptr_parent;
    ptr_child->depth = ptr_parent->depth + 1;

    (++ptr_child)->Reset(); // (1,1) (bot right)
    MAKE_LEAF_P(ptr_child);
    ptr_child->rect.tl.x = center_x;
    ptr_child->rect.tl.y = center_y;
    ptr_child->rect.br.x = rect.br.x;
    ptr_child->rect.br.y = rect.br.y;
    ptr_child->parent = ptr_parent;
    ptr_child->depth = ptr_parent->depth + 1;
  }

  void Quadtree::SearchNearestNeighborPrivate()
  {
    const auto &normalized_x = query_data_.normalized_x;
    const auto &normalized_y = query_data_.normalized_y;
    NodePtr &matched_node_ptr = query_data_.matched_node_ptr;

    // Initialize NN parameters
    ResetNNParameters();

    // Start from the root node.
    // Do Breadth First Search (BFS)
    simple_stack_.push(root_node_);
    while (!simple_stack_.empty())
    {
      NodePtr node_ptr = simple_stack_.topAndPop();
      // Current depth <= max_depth_ ??

      // If leaf node, find nearest point and 'BWBTest()'
      // if(nd.isLeaf()){
      if (IS_LEAF_P(node_ptr))
      {
        simple_stack_.addTotalAccess();
        if (FindNearestElement(normalized_x, normalized_y, node_ptr))
          matched_node_ptr = node_ptr;

        if (CheckBallWithinBound(normalized_x, normalized_y, node_ptr->rect, query_data_.min_distance_))
          break; // the nearest point is inside the node.
      }
      else
      { // this is not a leaf node.
        simple_stack_.addTotalAccess();

        // Go to child. Find most probable child first.
        NodePtr ptr_child = node_ptr->first_child;
        if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
        {
          simple_stack_.push(ptr_child);
        }
        ++ptr_child;
        if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
        {
          simple_stack_.push(ptr_child);
        }
        ++ptr_child;
        if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
        {
          simple_stack_.push(ptr_child);
        }
        ++ptr_child;
        if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
        {
          simple_stack_.push(ptr_child);
        }
      }
    }
  }

  void Quadtree::SearchNearestNeighborWithNodeCachePrivate()
  {
    // In this function, search the nearest element from the scratch (from the root node)
    const auto &normalized_x = query_data_.normalized_x;
    const auto &normalized_y = query_data_.normalized_y;

    // Initialize NN parameters
    ResetNNParameters();

    // Start from the cached node.
    // The cached node should be a leaf node.
    // If the cached node is invalid or the root node, normal NN search is executed.
    if (query_data_.cached_node_ptr == root_node_)
    {
      SearchNearestNeighborPrivate();
      return;
    }
    else // cached node is neither a root node nor a invalid node.
    {
      query_data_.matched_node_ptr = query_data_.cached_node_ptr;
      simple_stack_.addTotalAccess();

      // Find nearest point in the cached node.
      FindNearestElement(normalized_x, normalized_y, query_data_.matched_node_ptr);
      if (CheckBallWithinBound(normalized_x, normalized_y, query_data_.matched_node_ptr->rect, query_data_.min_distance_))
        return; // the nearest point is inside the cached node.

      // Go up to the until satisfying 'BWBTest == true'
      NodePtr node_ptr = query_data_.cached_node_ptr->parent;
      while (true)
      {
        simple_stack_.addTotalAccess();

        if (node_ptr == root_node_)
        {
          break; // reach the root node.
        }
        // If 'BWBTest' is passed on this node, the nearest one should be within
        // the chilren of this node.
        if (CheckBallWithinBound(normalized_x, normalized_y, node_ptr->rect, query_data_.min_distance_))
          break;

        node_ptr = node_ptr->parent;
      }

      // Start from the root node.
      // Do Breadth First Search (BFS)
      simple_stack_.push(node_ptr);
      while (!simple_stack_.empty())
      {
        NodePtr node_ptr = simple_stack_.topAndPop();
        // Current depth <= max_depth_ ??

        // If leaf node, find nearest point and 'BWBTest()'
        // if(nd.isLeaf()){
        if (IS_LEAF_P(node_ptr))
        {
          simple_stack_.addTotalAccess();
          if (FindNearestElement(normalized_x, normalized_y, node_ptr))
          {
            query_data_.matched_node_ptr = node_ptr;
          }

          if (CheckBallWithinBound(normalized_x, normalized_y, node_ptr->rect, query_data_.min_distance_))
            break; // the nearest point is inside the node.
        }
        else
        { // this is not a leaf node.
          simple_stack_.addTotalAccess();

          // Go to child. Find most probable child first.
          NodePtr ptr_child = node_ptr->first_child;
          if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
          {
            simple_stack_.push(ptr_child);
          }
          ++ptr_child;
          if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
          {
            simple_stack_.push(ptr_child);
          }
          ++ptr_child;
          if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
          {
            simple_stack_.push(ptr_child);
          }
          ++ptr_child;
          if (IS_ACTIVATED_P(ptr_child) && CheckBallOverlapBound(normalized_x, normalized_y, ptr_child->rect, query_data_.min_distance_))
          {
            simple_stack_.push(ptr_child);
          }
        }
      }
    }
  }

  inline bool Quadtree::CheckBallWithinBound(const float x, const float y, const Rect_u &rect, const float radius)
  {
    // Ball Within Bound (Circle bounded by rect check)
    const auto rect_size = static_cast<float>(rect.br.x - rect.tl.x);
    return (rect_size >= 2 * radius) && INBOUND_RECT_PTS(x, y, rect.tl.x + radius, rect.tl.y + radius, rect.br.x - radius, rect.br.y - radius);
  }

  inline bool Quadtree::CheckBallOverlapBound(const float x, const float y, const Rect_u &rect, const float radius)
  {
    // Ball Overlap Bound (Circle-rect collision check)
    const auto rect_center_x = (rect.br.x + rect.tl.x) * 0.5f;
    const auto rect_center_y = (rect.br.y + rect.tl.y) * 0.5f;
    const auto rect_halfsize_x = (rect.br.x - rect.tl.x) * 0.5f;
    const auto rect_halfsize_y = (rect.br.y - rect.tl.y) * 0.5f;

    const auto dx = fabs(x - rect_center_x);
    const auto dy = fabs(y - rect_center_y);

    if (dx > (rect_halfsize_x + radius))
      return false;
    if (dy > (rect_halfsize_y + radius))
      return false;

    if (dx <= rect_halfsize_x)
      return true;
    if (dy <= rect_halfsize_y)
      return true;

    const auto corner_dist_sq = (dx - rect_halfsize_x) * (dx - rect_halfsize_x) + (dy - rect_halfsize_y) * (dy - rect_halfsize_y);

    return (corner_dist_sq <= (radius * radius));
  }

  inline void Quadtree::MakeThisAsBranch(NodePtr node_ptr)
  {
    MAKE_BRANCH_P(node_ptr);
    node_ptr->element_head = nullptr;
    node_ptr->element_tail = nullptr;
  }

  uint32_t Quadtree::GetNumActivatedNodes()
  {
    return n_node_activated_;
  }

  void Quadtree::GetAllElementInRoot()
  {
    LinkedElementPtr elem_tmp = root_node_->element_head;
    while (elem_tmp != nullptr)
    {
      elem_tmp = elem_tmp->next;
    }
  }
};