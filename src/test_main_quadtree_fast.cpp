#include <iostream>
#include <memory>
#include <vector>

#include <random>
#include <ctime>
#include <functional>

#include "quadtree_fast.h"

void print_start();

int main() {
    print_start();

    clock_t start, finish;
    std::mt19937 engine((unsigned int)time(NULL));
    std::uniform_real_distribution<> distribution(1.0, 771.0);
    auto generator = std::bind(distribution, engine);

    float x_range[2] = {0.f,1032.f};
    float y_range[2] = {0.f,772.f};
    uint32_t max_depth = 7;

    try{
        std::shared_ptr<Quadtree> qt = nullptr;
        qt = std::make_shared<Quadtree>(x_range[0],x_range[1],y_range[0], y_range[1], max_depth);
        
        std::vector<std::pair<float,float>> points;
        std::vector<uint32_t> ids_node_matched;
        int n_pts = 500;
        for(int i = 0; i<n_pts;++i){
            points.push_back(std::make_pair<float,float>(generator(),generator()));
        }
        ids_node_matched.resize(n_pts);

        // Insert points
        for(int i = 0; i < n_pts; ++i){
            auto it = points[i];
            qt->insert(it.first,it.second, i);
        }
        std::cout << "insert OK!\n\n\n" <<std::endl;

        // Matching
        for(int i = 0; i < n_pts; ++i){
            ids_node_matched[i] = qt->NNSearch(points[i].first, points[i].second);
        }
        std::cout <<"Normal NN OK!\n\n\n" << std::endl;
        
        // cached Matching
        for(int i = 0; i < n_pts; ++i){
            ids_node_matched[i] = qt->cachedNNSearch(points[i].first, points[i].second, ids_node_matched[i]);
        }
        std::cout <<"Cached NN OK!\n\n\n" << std::endl;

    }
    catch (std::exception& e){
        std::cout <<"EXCEPTION: " << e.what() << std::endl;
    }

    return 0;
}


void print_start()
{
    printf("\n----------------------------\n");
    printf("-                          -\n");
    printf("-      program starts.     -\n");
    printf("-                          -\n");
    printf("----------------------------\n\n\n");
}
