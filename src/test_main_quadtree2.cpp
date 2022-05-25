#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <random>
#include <ctime>
#include <functional>
#include "quadtree2.h"

void print_start();

int main() {
    print_start();

    clock_t start, finish;
    std::mt19937 engine((unsigned int)time(NULL));
    std::uniform_real_distribution<> distribution(100.0, 500.0);
    auto generator = std::bind(distribution, engine);

    Quadtree qt;

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
