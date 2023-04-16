#include "quadtree/quadtree_array.h"

namespace ArrayBased
{
    Quadtree::Quadtree(
        float x_min, float x_max,
        float y_min, float y_max,
        size_t max_depth, size_t max_elem_per_leaf,
        float approx_rate, uint8_t flag_adj)
        : max_depth_(max_depth), x_range_{x_min, x_max}, y_range_{y_min, y_max},
          n_nodes_activated_(1), max_elements_per_leaf_(max_elem_per_leaf)
    {
        std::cout << "sizeof(Node): " << sizeof(Quadtree::Node) << "Bytes" << std::endl;

        // Make nodes
        n_nodes_total_ = (std::pow(4, max_depth_ + 1) - 1) / (4 - 1);
        nodes_.resize(n_nodes_total_ + 1);

        // Initialize a root node.
        MAKE_LEAF(nodes_[1]);

        // Initialize node elements.
        nodes_bin_.resize(nodes_.size());

        // Root size. (never goes to depth 14...)
        _numeric_uint br_x = std::numeric_limits<_numeric_uint>::max() >> 1;
        _numeric_uint br_y = std::numeric_limits<_numeric_uint>::max() >> 1;
        std::cout << "br_x, y: " << br_x << ", " << br_y << std::endl;

        nodes_[1].rect.tl = Corner<_numeric_uint>(0, 0);
        nodes_[1].rect.br = Corner<_numeric_uint>(br_x, br_y);

        // Calculate normalizers
        float x_normalizer_ = static_cast<float>(nodes_[1].rect.br.x) / (x_range_[1] - x_range_[0]);
        float y_normalizer_ = static_cast<float>(nodes_[1].rect.br.y) / (y_range_[1] - y_range_[0]);

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
        ID index_node = 1;
        nodes_[1].depth = 0;

        while (true)
        {
            Node &nd = nodes_[index_node]; // current node.
            const int8_t &depth = nd.depth;
            NodeBin &ndelems = nodes_bin_[index_node];

            if (IS_BRANCH(nd))
            {
                // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
                Flag flag_sn, flag_ew;
                FIND_QUADRANT(x_nom, y_nom, nd.rect, flag_sn, flag_ew);
                ID id_child = GET_CHILD_ID_FLAGS(index_node, flag_sn, flag_ew);

                nodes_[id_child].depth = depth + 1;
                index_node = id_child;
            }
            else if (IS_LEAF(nd))
            { // This is a leaf node.
                addDataToNode(index_node, id_elem);

                if (depth < max_depth_)
                { // nonmax depth.
                    int n_elem = getNumElemOfNode(index_node);
                    if (n_elem > max_elements_per_leaf_)
                    { // too much data. divide.
                        // Make all child to leaf (with no element)
                        ID id_child = GET_FIRST_CHILD_ID(index_node);
                        makeChildrenLeaves(id_child, nd.rect);
                        n_nodes_activated_ += 4;

                        // Do divide.
                        for (const ID &id_input_tmp : ndelems.elem_ids)
                        {
                            Flag flag_sn, flag_ew;
                            InputData &elem_tmp = input_points_[id_input_tmp];
                            FIND_QUADRANT(elem_tmp.x_nom, elem_tmp.y_nom, nd.rect, flag_sn, flag_ew);
                            ID id_child = GET_CHILD_ID_FLAGS(index_node, flag_sn, flag_ew);
                            nodes_[id_child].depth = depth + 1;
                            addDataToNode(id_child, id_input_tmp);
                        }

                        // make this node as a branch
                        makeBranch(index_node);
                    }
                }
                break;
            }
        }
    }

    inline void Quadtree::addDataToNode(ID id_node, ID id_elem)
    {
        nodes_bin_[id_node].elem_ids.push_back(id_elem);
    }

    inline int Quadtree::getNumElemOfNode(ID id_node)
    {
        return nodes_bin_[id_node].elem_ids.size();
    }

    inline void Quadtree::makeChildrenLeaves(ID id_child, const Rectangle<_numeric_uint> &rect)
    {
        _numeric_uint cent_x = (rect.tl.x + rect.br.x) >> 1;
        _numeric_uint cent_y = (rect.tl.y + rect.br.y) >> 1;

        MAKE_LEAF(nodes_[id_child]); // (0,0) (top left)
        nodes_[id_child].rect.tl.x = rect.tl.x;
        nodes_[id_child].rect.tl.y = rect.tl.y;
        nodes_[id_child].rect.br.x = cent_x;
        nodes_[id_child].rect.br.y = cent_y;

        MAKE_LEAF(nodes_[++id_child]); // (0,1) (top right)
        nodes_[id_child].rect.tl.x = cent_x;
        nodes_[id_child].rect.tl.y = rect.tl.y;
        nodes_[id_child].rect.br.x = rect.br.x;
        nodes_[id_child].rect.br.y = cent_y;

        MAKE_LEAF(nodes_[++id_child]); // (1,0) (bot left)
        nodes_[id_child].rect.tl.x = rect.tl.x;
        nodes_[id_child].rect.tl.y = cent_y;
        nodes_[id_child].rect.br.x = cent_x;
        nodes_[id_child].rect.br.y = rect.br.y;

        MAKE_LEAF(nodes_[++id_child]); // (1,1) (bot right)
        nodes_[id_child].rect.tl.x = cent_x;
        nodes_[id_child].rect.tl.y = cent_y;
        nodes_[id_child].rect.br.x = rect.br.x;
        nodes_[id_child].rect.br.y = rect.br.y;
    }

    inline void Quadtree::makeBranch(ID id_node)
    {
        MAKE_BRANCH(nodes_[id_node]);
        nodes_bin_[id_node].reset();
    }

    inline bool Quadtree::BWBTest(float x, float y, const Rectangle<_numeric_uint> &rect, float radius)
    {
        // Ball Within Bound (Circle bounded by rect check)
        float rect_size = (float)(rect.br.x - rect.tl.x);
        return (rect_size >= 2.0f * radius) && INBOUND_RECT_PTS(x, y, rect.tl.x + radius, rect.tl.y + radius, rect.br.x - radius, rect.br.y - radius);
    }

    inline bool Quadtree::BOBTest(float x, float y, const Rectangle<_numeric_uint> &rect, float R)
    {
        // Ball Overlap Bound (Circle-rect collision check)
        float rect_center_x = (float)(rect.br.x + rect.tl.x) * 0.5f;
        float rect_center_y = (float)(rect.br.y + rect.tl.y) * 0.5f;
        float rect_halfsize_x = (float)(rect.br.x - rect.tl.x) * 0.5f;
        float rect_halfsize_y = (float)(rect.br.y - rect.tl.y) * 0.5f;

        float dx = fabs(x - rect_center_x);
        float dy = fabs(y - rect_center_y);

        if (dx > (rect_halfsize_x + R))
            return false;
        if (dy > (rect_halfsize_y + R))
            return false;

        if (dx <= rect_halfsize_x)
            return true;
        if (dy <= rect_halfsize_y)
            return true;

        float corner_dist_sq = (dx - rect_halfsize_x) * (dx - rect_halfsize_x) + (dy - rect_halfsize_y) * (dy - rect_halfsize_y);

        return (corner_dist_sq <= (R * R));
    }

    void Quadtree::resetNNParameters()
    {
        // Initialize values
        simple_stack_.clear();
        simple_stack_.clearTotalAccess();
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_ = std::numeric_limits<float>::max();
    }

    void Quadtree::nearestNeighborSearchPrivate()
    {
        // In this function, search the nearest element from the scratch (from the root node)
        float x_nom = query_data_.x_nom;
        float y_nom = query_data_.y_nom;
        ID &id_point_matched = query_data_.id_point_matched;
        ID &id_node_matched = query_data_.id_node_matched;

        // Initialize NN parameters
        resetNNParameters();

        // Start from the root node.
        // Do Breadth First Search (BFS)
        simple_stack_.push(1);
        while (!simple_stack_.empty())
        {
            ID id_node = simple_stack_.topAndPop();
            Node &nd = nodes_[id_node];
            // Current depth <= max_depth_ ??

            // If leaf node, find nearest point and 'BWBTest()'
            // if(nd.isLeaf()){
            if (IS_LEAF(nd))
            {
                simple_stack_.addTotalAccess();
                NodeBin &nd_elems = nodes_bin_[id_node];
                if (findNearestElem(x_nom, y_nom, nd_elems))
                {
                    id_node_matched = id_node;
                }

                if (BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_))
                    break; // the nearest point is inside the node.
            }
            else
            { // this is not a leaf node.
                simple_stack_.addTotalAccess();

                // Go to child. Find most probable child first.
                ID id_child = GET_FIRST_CHILD_ID(id_node);
                if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                {
                    simple_stack_.push(id_child);
                }
                ++id_child;
                if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                {
                    simple_stack_.push(id_child);
                }
            }
        }
    }

    bool Quadtree::findNearestElem(float x, float y, const NodeBin &elems_thisnode)
    {
        bool findNewNearest = false;
        for (const ID &id_elem : elems_thisnode.elem_ids)
        {
            InputData &elem = input_points_[id_elem];
            float dist_temp = DIST_EUCLIDEAN(x, y, elem.x_nom, elem.y_nom);
            if (dist_temp < query_data_.min_dist2_)
            {
                this->query_data_.id_point_matched = elem.id_point;
                this->query_data_.id_elem_matched = id_elem;
                this->query_data_.min_dist2_ = dist_temp * params_.approx_rate;
                this->query_data_.min_dist_ = sqrt(query_data_.min_dist2_);
                findNewNearest = true;
            }
        }
        return findNewNearest;
    }

    void Quadtree::cachedNearestNeighborSearchPrivate()
    {
        // In this function, search the nearest element from the scratch (from the root node)
        float &x_nom = query_data_.x_nom;
        float &y_nom = query_data_.y_nom;
        ID &id_point_matched = query_data_.id_point_matched;

        resetNNParameters(); // Initialize NN parameters

        // Start from the cached node.
        // The cached node should be a leaf node.
        // If the cached node is invalid or the root node, normal NN search is executed.
        if (query_data_.id_node_cached <= 1)
        {
            nearestNeighborSearchPrivate();
            return;
        }
        else // cached node is neither a root node nor a invalid node.
        {
            query_data_.id_node_matched = query_data_.id_node_cached;
            simple_stack_.addTotalAccess();

            // Find nearest point in the cached node.
            findNearestElem(x_nom, y_nom, nodes_bin_[query_data_.id_node_matched]);
            if (BWBTest(x_nom, y_nom, nodes_[query_data_.id_node_matched].rect, query_data_.min_dist_))
            {
                return; // the nearest point is inside the cached node.
            }

            // Go up to the until satisfying 'BWBTest == true'
            ID id_node = GET_PARENT_ID(query_data_.id_node_cached);
            while (true)
            {
                simple_stack_.addTotalAccess();

                if (id_node <= 1)
                {
                    id_node = 1;
                    break; // reach the root node.
                }
                // If 'BWBTest' is passed on this node, the nearest one should be within
                // the chilren of this node.
                if (BWBTest(x_nom, y_nom, nodes_[id_node].rect, query_data_.min_dist_))
                    break;

                id_node = GET_PARENT_ID(id_node);
            }

            // Start from the root node.
            // Do Breadth First Search (BFS)
            simple_stack_.push(id_node);
            while (!simple_stack_.empty())
            {
                id_node = simple_stack_.topAndPop();
                Node &nd = nodes_[id_node];

                // If leaf node, find nearest point and 'BWBTest()'
                // if(nd.isLeaf()){
                if (IS_LEAF(nd))
                {
                    simple_stack_.addTotalAccess();
                    NodeBin &nd_elems = nodes_bin_[id_node];
                    if (findNearestElem(x_nom, y_nom, nd_elems))
                    {
                        query_data_.id_node_matched = id_node;
                    }

                    if (BWBTest(x_nom, y_nom, nd.rect, query_data_.min_dist_))
                        break; // the nearest point is inside the node.
                }
                else
                { // this is not a leaf node.
                    simple_stack_.addTotalAccess();

                    // Go to child. Find most probable child first.
                    ID id_child = GET_FIRST_CHILD_ID(id_node);
                    if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                    {
                        simple_stack_.push(id_child);
                    }
                    ++id_child;
                    if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                    {
                        simple_stack_.push(id_child);
                    }
                    ++id_child;
                    if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                    {
                        simple_stack_.push(id_child);
                    }
                    ++id_child;
                    if (IS_ACTIVATED(nodes_[id_child]) && BOBTest(x_nom, y_nom, nodes_[id_child].rect, query_data_.min_dist_))
                    {
                        simple_stack_.push(id_child);
                    }
                }
            }
        }
    }

    void Quadtree::NNSearch(float x, float y,
                            ID &id_point_matched, ID &id_node_matched)
    {
        query_data_.x = x;
        query_data_.y = y;
        query_data_.x_nom = x * normalizer_;
        query_data_.y_nom = y * normalizer_;
        query_data_.id_node_cached = 1; // non-cached case now.

        nearestNeighborSearchPrivate();

        id_point_matched = query_data_.id_point_matched;
        id_node_matched = query_data_.id_node_matched;
    }

    void Quadtree::NNSearchDebug(float x, float y,
                                 ID &id_point_matched, ID &id_node_matched, size_t &n_access)
    {
        NNSearch(x, y, id_point_matched, id_node_matched);
        n_access = simple_stack_.getTotalAccess();
    }

    void Quadtree::cachedNNSearch(float x, float y, int id_node_cached,
                                  ID &id_point_matched, ID &id_node_matched)
    {
        query_data_.x = x;
        query_data_.y = y;
        query_data_.x_nom = x * normalizer_;
        query_data_.y_nom = y * normalizer_;
        query_data_.id_node_cached = id_node_cached;

        cachedNearestNeighborSearchPrivate();

        id_point_matched = query_data_.id_point_matched;
        id_node_matched = query_data_.id_node_matched;
    }

    void Quadtree::cachedNNSearchDebug(float x, float y, int id_node_cached,
                                       ID &id_point_matched, ID &id_node_matched, size_t &n_access)
    {
        cachedNNSearch(x, y, id_node_cached, id_point_matched, id_node_matched);
        n_access = simple_stack_.getTotalAccess();
    }

    inline void Quadtree::resetQueryData()
    {
        query_data_.id_point_matched = 0;
        query_data_.id_elem_matched = 0;
        query_data_.id_node_cached = 1;
        query_data_.id_node_matched = 1;
        query_data_.min_dist2_ = std::numeric_limits<float>::max();
        query_data_.min_dist_ = std::numeric_limits<float>::max();
        query_data_.x = -1.0f;
        query_data_.y = -1.0f;
        query_data_.x_nom = -1.0f;
        query_data_.y_nom = -1.0f;
    }

    const size_t Quadtree::getNumNodesTotal() const
    {
        return n_nodes_total_;
    }
    const size_t Quadtree::getNumNodesActivated() const
    {
        return n_nodes_activated_;
    }
};
