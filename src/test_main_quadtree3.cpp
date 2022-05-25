#include <iostream>
#include <memory>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <random>
#include <ctime>
#include <functional>

#include "quadtree3.h"

void print_start();

int main() {
    print_start();

    clock_t start, finish;
    std::mt19937 engine((unsigned int)time(NULL));
    std::uniform_real_distribution<> distribution(100.0, 500.0);
    auto generator = std::bind(distribution, engine);

    float x_range[2] = {0,1032};
    float y_range[2] = {0,772};
    uint32_t max_depth = 12;

    try{
        std::shared_ptr<Quadtree> qt = nullptr;
        qt = std::make_shared<Quadtree>(x_range[0],x_range[1],y_range[0], y_range[1], max_depth);
        
        qt->insert(0, 1.64, 1.5);
        qt->insert(0, 1.64, 1.5);
        qt->insert(1, 555.64, 555.5);
        qt->insert(2, 1.64, 555.5);
        qt->insert(3, 555.64, 1.5);
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
