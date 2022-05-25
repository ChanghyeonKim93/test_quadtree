#include "quadtree2.h"

Quadtree::Quadtree(){
    max_depth_ = 8;

    root_rect.tl = QuadVertex(0,0);
    root_rect.br = QuadVertex(std::pow(2, max_depth_-1),std::pow(2, max_depth_-1));

    std::cout << root_rect <<std::endl;

    // Push the root node.   
    nodes.push_back(QuadNode());

    std::cout << "# of nodes: " << 1*(std::pow(4,max_depth_)-1)/3 << std::endl;

};

Quadtree::~Quadtree(){

};

// QuadNodeList Quadtree::findAllLeaves(){
    
//     QuadNodeList leaves, to_process;
//     to_process.push_back(nodes[0]);

//     while(to_process.size() > 0){
//         const QuadNode nd = (const QuadNode)to_process.back();
//         to_process.pop_back();

//         // If this node is a leaf, insert it to the list
//         if(nodes[nd.index].count != -1){ // 
//             leaves.push_back(nd);
//         }
//         else{
//             // Otherwise push the children that intersect the rectangle.

//             nd.
//         }
//     }

//     return leaves;
// };


