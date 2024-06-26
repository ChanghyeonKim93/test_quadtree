#include <iostream>
#include <memory>
#include <vector>

#include <functional>
#include <random>

#include "quadtree_fast.h"
#include "timer.h"

int main() {
  std::mt19937 engine((unsigned int)time(NULL));
  std::uniform_real_distribution<> distribution(100.0, 600.0);
  auto generator = std::bind(distribution, engine);

  float x_range[2] = {0.f, 1032.f};
  float y_range[2] = {0.f, 772.f};
  uint32_t max_depth = 8;
  uint32_t max_elem_per_leaf = 20;
  int n_pts = 5000;

  try {
    std::shared_ptr<Quadtree> qt = nullptr;
    qt = std::make_shared<Quadtree>(x_range[0], x_range[1], y_range[0],
                                    y_range[1], max_depth, max_elem_per_leaf);

    std::vector<std::pair<float, float>> points;
    std::vector<uint32_t> ids_node_matched;
    for (int i = 0; i < n_pts; ++i) {
      points.push_back(std::make_pair<float, float>(generator(), generator()));
    }
    ids_node_matched.resize(n_pts);

    // Insert points
    std::cout << "start insert..." << std::endl;
    timer::tic();
    for (int i = 0; i < n_pts; ++i) {
      auto it = points[i];
      qt->insert(it.first, it.second, i);
    }
    std::cout << "insert OK! time: " << timer::toc(0) << " ms" << std::endl;

    // Time Test...
    std::vector<float> time_normal;
    std::vector<float> time_cached;
    std::vector<uint32_t> access_normal;
    std::vector<uint32_t> access_cached;
    int n_steps = 50;
    access_normal.resize(n_steps);
    access_cached.resize(n_steps);
    std::cout << "start normal matching..." << std::endl;
    for (int ii = 0; ii < n_steps; ++ii) {
      // Matching
      float x_step = 0.1 * (float)ii;
      float y_step = 0.1 * (float)ii;
      timer::tic();
      for (int i = 0; i < n_pts; i += 5) {
        uint32_t access_temp = 0;
        ids_node_matched[i] = qt->NNSearchDebug(
            points[i].first + x_step, points[i].second + y_step, access_temp);
        access_normal[ii] += access_temp;
      }
      time_normal.push_back(timer::toc(0));
      // std::cout << ii <<"/" << n_steps <<std::endl;
      // std::cout << "Normal NN OK! time: " << timer::toc(0) << " ms"
      // <<std::endl;
    }
    std::cout << "normal matching done." << std::endl;

    for (int i = 0; i < n_pts; i += 1) {
      ids_node_matched[i] = qt->NNSearch(points[i].first, points[i].second);
    }
    std::cout << "start cached matching..." << std::endl;
    for (int ii = 0; ii < n_steps; ++ii) {
      // cached Matching
      float x_step = 0.5 * (float)ii;
      float y_step = 0.5 * (float)ii;
      timer::tic();
      for (int i = 0; i < n_pts; i += 5) {
        uint32_t access_temp = 0;
        ids_node_matched[i] = qt->cachedNNSearchDebug(
            points[i].first + x_step, points[i].second + y_step,
            ids_node_matched[i], access_temp);
        access_cached[ii] += access_temp;
      }
      time_cached.push_back(timer::toc(0));
      // std::cout << ii <<"/" << n_steps <<std::endl;
      // std::cout << "Cached NN OK! time: " << timer::toc(0) << " ms"
      // <<std::endl;
    }
    std::cout << "cached matching done." << std::endl;

    // Show the test results
    for (int ii = 0; ii < n_steps; ++ii) {
      std::cout << ii << "-th normal/cached: " << time_normal[ii] << ", "
                << time_cached[ii] << " / ";
      std::cout << "access normal/cached: " << access_normal[ii] << ", "
                << access_cached[ii] << std::endl;
    }
  } catch (std::exception& e) {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  return 0;
}