#include "quadtree/quadtree_hash2.h"

namespace HashBased2
{
  Quadtree::Quadtree(
    float x_min, float x_max,
    float y_min, float y_max,
    size_t max_depth, size_t max_elem_per_leaf,
    float approx_rate, uint8_t flag_adj)
    : max_depth_(max_depth), x_range_{x_min, x_max}, y_range_{y_min, y_max},
      n_nodes_activated_(0), max_elements_per_leaf_(max_elem_per_leaf)
  {
    std::cout << "sizeof(Node): " << sizeof(Quadtree::Node) << std::endl;
    node_list_.reserve(10000);
    node_elem_id_list_.reserve(10000);

    const uint32_t node_id_root = 1;
    const uint8_t depth_root = 0;

    // Root size. (never goes to depth 14...)
    _numeric_uint br_x = std::numeric_limits<_numeric_uint>::max() >> 1;
    _numeric_uint br_y = std::numeric_limits<_numeric_uint>::max() >> 1;
    std::cout << "br_x, y: " << br_x << ", " << br_y << std::endl;
    Rectangle<_numeric_uint> rect_root =
      Rectangle<_numeric_uint>(Corner<_numeric_uint>(0, 0), Corner<_numeric_uint>(br_x, br_y));
    
    this->addNewNode(node_id_root, depth_root, rect_root); // root node

    // Calculate normalizers
    float x_normalizer_ = static_cast<float>(rect_root.br.x) / (x_range_[1] - x_range_[0]);
    float y_normalizer_ = static_cast<float>(rect_root.br.y) / (y_range_[1] - y_range_[0]);

    if (x_normalizer_ > y_normalizer_)
      normalizer_ = x_normalizer_;
    else
      normalizer_ = y_normalizer_;
    
    // Parameter setting (clipping approximating rate)
    params_.approx_rate = approx_rate;
    if (params_.approx_rate > 1.0f)
        params_.approx_rate = 1.0f;
    if (params_.approx_rate < 0.5f)
        params_.approx_rate = 0.5f;
    params_.approx_rate = params_.approx_rate * params_.approx_rate;

    params_.flag_adj_search_only = flag_adj;
  }

  Quadtree::~Quadtree()
  { 
    std::cout << "Quadtree is deleted.\n"; 
  }
  
  void Quadtree::insert(float x, float y, int id_point)
  {
    if (x > x_range_[0] && x < x_range_[1] && y > y_range_[0] && y < y_range_[1])
    {
      // normalize input point coordinates
      float x_nom = x * normalizer_;
      float y_nom = y * normalizer_;

      // Add the input data as an InputData object
      input_points_.emplace_back(x_nom, y_nom, static_cast<ID>(id_point));

      // Insert into the quadtree.
      this->insertPrivateStack(x_nom, y_nom, input_points_.size() - 1); // Stack-based approach (DFS), faster than Recursion-based one 1.2 times.
    }
    else
      throw std::runtime_error("Input x,y is out of the quadtree range.");
  }

  void Quadtree::insertPrivateStack(const float x_nom, const float y_nom, const ID id_elem)
  {
    ID node_id_to_visit = 1;

    while (true)
    {
      const Node &node = node_list_.at(node_id_to_visit); // current node.
      const Rectangle<uint16_t>& rect_to_visit = node_list_[node_id_to_visit].getRect();
      const uint8_t depth = node.getDepth();

      if (node.isBranch())
      {
        // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
        Flag flag_sn, flag_ew;
        FIND_QUADRANT(x_nom, y_nom, rect_to_visit, flag_sn, flag_ew);
        ID child_id = GET_CHILD_ID_FLAGS(node_id_to_visit, flag_sn, flag_ew);

        if(node_list_.count(child_id) == 0){
          this->addNewNode(child_id, depth + 1, this->getSubRectangle(rect_to_visit, flag_sn, flag_ew));
        }

        node_id_to_visit = child_id;
      }
      else if (node.isLeaf())
      {
        // This is a leaf node.
        this->addDataToNode(node_id_to_visit, id_elem);

        if (depth < this->max_depth_)
        { 
          // nonmax depth.
          const size_t n_elem = this->getNumElemOfNode(node_id_to_visit);
          if (n_elem > this->max_elements_per_leaf_)
          { 
            // too much data. divide.
            // Make all child to leaf (with no element)
            // Do divide.
            for (const ID &elem_id : node_elem_id_list_[node_id_to_visit])
            {
              Flag flag_sn, flag_ew;
              InputData &elem = input_points_[elem_id];
              FIND_QUADRANT(elem.x_nom, elem.y_nom, rect_to_visit, flag_sn, flag_ew);
              ID child_id = GET_CHILD_ID_FLAGS(node_id_to_visit, flag_sn, flag_ew);
              if(node_elem_id_list_.count(child_id) == 0){
                this->addNewNode(child_id, depth + 1, this->getSubRectangle(rect_to_visit, flag_sn, flag_ew));
              }
              this->addDataToNode(child_id, elem_id);
            }
            // Delete capacity
            std::vector<uint32_t> vt;
            vt.swap(node_elem_id_list_[node_id_to_visit]);

            // make this node as a branch
            this->makeBranch(node_id_to_visit);
          }
        }
        break;
      }
    }
  }

  void Quadtree::addNewNode(const uint32_t node_id, const uint8_t depth, const Rectangle<uint16_t>& rect)
  {
    node_elem_id_list_[node_id] = std::vector<uint32_t>();
    node_elem_id_list_[node_id].reserve(max_elements_per_leaf_ + 10);
    node_list_[node_id] = Node();
    Node& node = node_list_[node_id];
    node.setDepth(depth);
    node.setRect(rect);
    node.makeLeaf();
    ++this->n_nodes_activated_;
  }

  Quadtree::Rectangle<uint16_t> Quadtree::getSubRectangle(const Quadtree::Rectangle<uint16_t>& rect, const bool flag_sn, const bool flag_ew)
  {
    Quadtree::Rectangle<uint16_t> rect_sub;

    if(flag_sn) { // 1: south
      rect_sub.tl.y = (rect.tl.y + rect.br.y) >> 1; 
      rect_sub.br.y = rect.br.y; 
    }
    else { // 0: north
      rect_sub.tl.y = rect.tl.y; 
      rect_sub.br.y = (rect.tl.y + rect.br.y) >> 1; 
    }

    if(flag_ew) { // 1: east
      rect_sub.tl.x = (rect.tl.x + rect.br.x) >> 1; 
      rect_sub.br.x = rect.br.x;
    } 
    else { // 0: west
      rect_sub.tl.x = rect.tl.x; 
      rect_sub.br.x = (rect.tl.x + rect.br.x) >> 1; 
    }
    return rect_sub;
  }

  inline void Quadtree::addDataToNode(ID node_id, ID elem_id)
  {
    node_elem_id_list_[node_id].push_back(elem_id);
  }
  
  inline size_t Quadtree::getNumElemOfNode(ID node_id)
  {
    return node_elem_id_list_[node_id].size();
  }

  inline void Quadtree::makeBranch(ID node_id){
    node_list_[node_id].makeBranch();
  }

  const size_t Quadtree::getNumNodesActivated() const
  {
      return n_nodes_activated_;
  }
};
