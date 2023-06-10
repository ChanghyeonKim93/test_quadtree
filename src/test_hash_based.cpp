#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

#include <random>
#include <functional>

#include "quadtree/quadtree_hash.h"
#include "timer.h"

using namespace HashBased;

int main() {
  std::mt19937 engine((unsigned int)time(NULL));
  std::uniform_real_distribution<> distribution(50.0, 590.0);
  auto generator = std::bind(distribution, engine);

  float x_range[2] = {0.f,640.f};
  float y_range[2] = {0.f,640.f};

  size_t max_depth         = 7;
  size_t max_elem_per_leaf = 20;
  float approx_rate = 1.0;
  int n_pts = 40000;
  int n_pts_q = 30000;

  for(int d = 0; d <= max_depth; ++d){
    std::shared_ptr<Quadtree> qt = nullptr;
    qt = std::make_shared<Quadtree>(
        x_range[0],x_range[1],y_range[0], y_range[1], d, max_elem_per_leaf,
        approx_rate);
  }              
  return -1;
}