#include "quadtree/quadtree_pointer.h"

namespace PointerBased{
    Quadtree::Quadtree(
        float x_min, float x_max,
        float y_min, float y_max,
        ObjectPool<QuadNode>* objpool_node, ObjectPool<Elem>* objpool_elem,
        uint32_t max_depth, uint32_t max_elem_per_leaf,
        float approx_rate, uint8_t flag_adj)
    : max_depth_(max_depth), x_range_{x_min,x_max}, y_range_{y_min,y_max},
    n_node_activated_(1), max_elem_per_leaf_(max_elem_per_leaf),
    objpool_node_(objpool_node), objpool_elem_(objpool_elem)
    {
        // Initialize a root node.
        root_node_ = new QuadNode();
        root_node_->depth = 0;

        // Root size.
        QuadUint br_x = 1 << 13;
        QuadUint br_y = 1 << 13;
        
        root_node_->rect.tl = Pos2d<QuadUint>(0, 0);
        root_node_->rect.br = Pos2d<QuadUint>(br_x, br_y);

        // Calculate normalizers 
        x_normalizer_ = (float)root_node_->rect.br.x/(x_range_[1] - x_range_[0]);
        y_normalizer_ = (float)root_node_->rect.br.y/(y_range_[1] - y_range_[0]);

        if(x_normalizer_ > y_normalizer_) y_normalizer_ = x_normalizer_;
        else x_normalizer_ = y_normalizer_;

        // Parameter setting
        params_.approx_rate = approx_rate;
        if(params_.approx_rate > 1.0f) params_.approx_rate = 1.0f;
        if(params_.approx_rate < 0.5f) params_.approx_rate = 0.5f;
        params_.approx_rate = params_.approx_rate*params_.approx_rate;

        params_.flag_adj_search_only = flag_adj;
    };

    Quadtree::~Quadtree(){
        delete root_node_;
        std::cout << "Quadtree is deleted.\n";
    }

    void Quadtree::insert(float x, float y, int id_data){
        if( x > x_range_[0] && x < x_range_[1]
        &&  y > y_range_[0] && y < y_range_[1] )
        {
            // normalize input point coordinates
            float x_nom = x*x_normalizer_; float y_nom = y*y_normalizer_;

            // Add the input data as an Element object
            ElemPtr elem_new = objpool_elem_->getObject();

            // Initialize the 'insert_data_' 
            // This is to avoid recursive copies of recursive call of function inputs) 
            insert_data_.setData(x_nom, y_nom, elem_new);

            // Insert query_data_ into the quadtree.
            // this->insertPrivate(1, 0); // Recursion-based approach (DFS)
            this->insertPrivateStack(); // Stack-based approach (DFS), faster than Recursion-based one 1.2 times.
        }
        else throw std::runtime_error("Input x,y is out of the quadtree range.");
    };

    void Quadtree::insertPrivateStack()
    {
        float& x_nom = insert_data_.x_nom;
        float& y_nom = insert_data_.y_nom;

        QuadNodePtr ptr_node = root_node_;
        ptr_node->depth = 0;
        while(true){
            int depth = ptr_node->depth;

            if(IS_BRANCH(*ptr_node)){
                // Child cases of this branch: 1) not activated, 2) branch, 3) leaf
                Flag flag_sn, flag_ew;
                FIND_QUADRANT(x_nom, y_nom, ptr_node->rect,      flag_sn, flag_ew);
                // uint8_t idx_child = GET

                ptr_node = ptr_node->first_child + idx_child;
                ptr_node->depth = depth + 1;
            }
            else if(IS_LEAF(*ptr_node)){
                addDataToNode();

                if(depth < max_depth_){
                    // make all child to leaf (with no element)
                    

                    // make this node as a branch
                    makeBranch(*ptr_node);
                }
                break;
            }
        }
    };

};