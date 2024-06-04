#include <iostream>
#include <memory>
#include <vector>

#include <functional>
#include <random>

#include "quadtree_fast2.h"
#include "timer.h"

int main() {
  std::mt19937 engine((unsigned int)time(NULL));
  std::uniform_real_distribution<> distribution(100.0, 600.0);
  auto generator = std::bind(distribution, engine);

  float x_range[2] = {0.f, 1032.f};
  float y_range[2] = {0.f, 772.f};
  uint32_t max_depth = 8;
  uint32_t max_elem_per_leaf = 300;
  int n_pts = 2000;

  try {
    std::vector<std::pair<float, float>> points;
    std::vector<uint32_t> ids_node_matched;
    for (int i = 0; i < n_pts; ++i) {
      points.push_back(std::make_pair<float, float>(generator(), generator()));
    }
    ids_node_matched.resize(n_pts);

    std::vector<float> time_insert(max_elem_per_leaf + 1);

    std::vector<float> time_normal(max_elem_per_leaf + 1);
    std::vector<float> time_cached(max_elem_per_leaf + 1);
    std::vector<uint32_t> access_normal(max_elem_per_leaf + 1);
    std::vector<uint32_t> access_cached(max_elem_per_leaf + 1);

    float x_step = 20.0;
    float y_step = 20.0;

    for (int d = 1; d <= max_elem_per_leaf; ++d) {
      std::shared_ptr<Quadtree> qt = nullptr;
      qt = std::make_shared<Quadtree>(x_range[0], x_range[1], y_range[0],
                                      y_range[1], max_depth, d);

      // Insert points
      std::cout << "start insert..." << std::endl;
      timer::tic();
      for (int i = 0; i < n_pts; ++i) {
        auto it = points[i];
        qt->insert(it.first, it.second, i);
      }
      time_insert[d] = timer::toc(0);

      // normal matching
      std::cout << "start normal matching..." << std::endl;
      timer::tic();
      for (int i = 0; i < n_pts; i += 5) {
        uint32_t access_temp = 0;
        ids_node_matched[i] = qt->NNSearchDebug(
            points[i].first + x_step, points[i].second + y_step, access_temp);
        access_normal[d] += access_temp;
      }
      time_normal[d] = timer::toc(0);

      // Cached matching
      std::cout << "start cached matching..." << std::endl;
      timer::tic();
      for (int i = 0; i < n_pts; i += 5) {
        uint32_t access_temp = 0;
        ids_node_matched[i] = qt->cachedNNSearchDebug(
            points[i].first + x_step, points[i].second + y_step,
            ids_node_matched[i], access_temp);
        access_cached[d] += access_temp;
      }
      time_cached[d] = timer::toc(0);
    }

    // Show the test results
    for (int d = 0; d <= max_elem_per_leaf; ++d) {
      std::cout << " max elem [" << d << "] insert: " << time_insert[d]
                << ", normal/cached: " << time_normal[d] << ", "
                << time_cached[d] << " / ";
      std::cout << "access normal/cached: " << access_normal[d] << ", "
                << access_cached[d] << std::endl;
    }
  } catch (std::exception& e) {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  return 0;
}