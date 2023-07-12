#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

#include <random>
#include <functional>

#include "quadtree/quadtree_pointer_refactor.h"
#include "timer.h"

using namespace quadtree;

int main()
{
  std::mt19937 engine((unsigned int)time(NULL));
  std::uniform_real_distribution<> distribution(50.0, 590.0);
  auto generator = std::bind(distribution, engine);

  float x_range[2] = {0.f, 640.f};
  float y_range[2] = {0.f, 640.f};

  int max_tree_depth = 12;
  int max_num_element_in_leaf = 20;
  float approx_rate = 1.0;
  int num_reference_point = 10000;
  int num_query_point = 1000;

  struct Pixel
  {
    float u{-1.0f};
    float v{-1.0f};
    Pixel(const float input_u, const float input_v) : u(input_u), v(input_v) {}
  };

  // Generate Reference Points
  std::vector<Pixel> reference_point_list;
  for (int i = 0; i < num_reference_point; ++i)
    reference_point_list.emplace_back(generator(), generator());

  // Generate Query Points
  std::vector<Pixel> query_point_list;
  for (int i = 0; i < num_query_point; ++i)
    query_point_list.emplace_back(generator(), generator());

  std::vector<float> true_min_distance_list;
  std::vector<uint32_t> true_matched_reference_id_list;
  true_min_distance_list.resize(num_query_point);
  true_matched_reference_id_list.resize(num_query_point);
  for (int i = 0; i < num_query_point; ++i)
  {
    const auto &query_point = query_point_list[i];
    float min_distance_squared = std::numeric_limits<float>::max();
    int data_id = 0;
    for (int j = 0; j < num_reference_point; ++j)
    {
      const auto &reference_point = reference_point_list[j];
      const auto distance = DIST_EUCLIDEAN(query_point.u, query_point.v,
                                           reference_point.u, reference_point.v);
      if (distance < min_distance_squared)
      {
        data_id = j;
        min_distance_squared = distance;
      }
    }

    true_min_distance_list[i] = min_distance_squared;
    true_matched_reference_id_list[i] = data_id;
  }

  // Matching Test
  std::vector<NodePtr> matched_node_ptr_list;
  std::vector<int> matched_reference_id_list;
  matched_node_ptr_list.resize(num_query_point);
  matched_reference_id_list.resize(num_query_point);
  try
  {
    std::vector<float> time_construct(max_tree_depth + 1);
    std::vector<float> time_insert(max_tree_depth + 1);

    std::vector<float> time_normal(max_tree_depth + 1);
    std::vector<float> time_cached(max_tree_depth + 1);
    std::vector<uint32_t> access_normal(max_tree_depth + 1);
    std::vector<double> min_access_normal(max_tree_depth + 1);
    std::vector<double> max_access_normal(max_tree_depth + 1);
    std::vector<double> avg_access_normal(max_tree_depth + 1);
    std::vector<double> std_access_normal(max_tree_depth + 1);

    std::vector<uint32_t> access_cached(max_tree_depth + 1);
    std::vector<double> min_access_cached(max_tree_depth + 1);
    std::vector<double> max_access_cached(max_tree_depth + 1);
    std::vector<double> avg_access_cached(max_tree_depth + 1);
    std::vector<double> std_access_cached(max_tree_depth + 1);

    std::vector<uint32_t> diff_normal(max_tree_depth + 1);
    std::vector<uint32_t> diff_cached(max_tree_depth + 1);

    for (int d = 0; d <= max_tree_depth; ++d)
    {
      timer::tic();
      std::shared_ptr<Quadtree> quadtree = nullptr;
      quadtree = std::make_shared<Quadtree>(
          x_range[0], x_range[1], y_range[0], y_range[1],
          d, max_num_element_in_leaf,
          approx_rate);
      time_construct[d] = timer::toc(0);

      // Insert points
      timer::tic();
      for (uint32_t i = 0; i < num_reference_point; ++i)
      {
        ID id_data = i;
        quadtree->InsertReferenceData(reference_point_list[i].u, reference_point_list[i].v, id_data);
      }
      std::cout << "# nodes - activated: "
                << quadtree->GetNumActivatedNodes() << std::endl;

      time_insert[d] = timer::toc(0);

      // normal matching
      std::vector<uint32_t> accesses;
      std::cout << "search starts" << std::endl;

      timer::tic();
      for (uint32_t i = 0; i < num_query_point; ++i)
      {
        uint32_t access_temp = 0;
        quadtree->SearchNearestNeighbor_debug(
            query_point_list[i].u, query_point_list[i].v,
            matched_reference_id_list[i], matched_node_ptr_list[i], access_temp);
        access_normal[d] += access_temp;
        accesses.push_back(access_temp);
      }
      time_normal[d] = timer::toc(0);

      std::cout << "search done" << std::endl;
      std::sort(accesses.begin(), accesses.end());
      min_access_normal[d] = accesses.front();
      max_access_normal[d] = accesses.back();
      double sum = std::accumulate(accesses.begin(), accesses.end(), 0.0);
      double mean = sum / accesses.size();
      double sq_sum = std::inner_product(accesses.begin(), accesses.end(), accesses.begin(), 0.0);
      double stdev = std::sqrt(sq_sum / accesses.size() - mean * mean);
      avg_access_normal[d] = mean;
      std_access_normal[d] = stdev;

      for (int i = 0; i < num_query_point; ++i)
      {
        uint32_t id_mat = matched_reference_id_list[i];
        uint32_t id_mat_true = true_matched_reference_id_list[i];

        if (id_mat != id_mat_true)
        {
          ++diff_normal[d];
          std::cout
              << "mismatched: " << id_mat << "," << id_mat_true << " / "
              << reference_point_list[id_mat_true].u << "," << reference_point_list[id_mat_true].v
              << " / " << reference_point_list[id_mat].u << "," << reference_point_list[id_mat].v
              << " / mindist2 (true,est):" << true_min_distance_list[i] << ","
              << DIST_EUCLIDEAN(reference_point_list[id_mat].u, reference_point_list[id_mat].v, query_point_list[i].u, query_point_list[i].v) << std::endl;
        }
      }

      // Cached matching
      accesses.resize(0);
      timer::tic();
      for (int i = 0; i < num_query_point; ++i)
      {
        uint32_t access_temp = 0;
        quadtree->SearchNearestNeighborWithNodeCache_debug(
            query_point_list[i].u, query_point_list[i].v, matched_node_ptr_list[i],
            matched_reference_id_list[i], matched_node_ptr_list[i], access_temp);
        access_cached[d] += access_temp;
        accesses.push_back(access_temp);
      }
      time_cached[d] = timer::toc(0);

      for (int i = 0; i < num_query_point; ++i)
      {
        uint32_t id_mat = matched_reference_id_list[i];
        uint32_t id_mat_true = true_matched_reference_id_list[i];

        if (id_mat != id_mat_true)
        {
          ++diff_cached[d];
          std::cout
              << "mismatched: " << id_mat << "," << id_mat_true << " / "
              << reference_point_list[id_mat].u << "," << reference_point_list[id_mat].v
              << " / " << reference_point_list[id_mat_true].u << "," << reference_point_list[id_mat_true].v
              << " / mindist2 (true,est):" << true_min_distance_list[i] << ","
              << DIST_EUCLIDEAN(reference_point_list[id_mat].u, reference_point_list[id_mat].v, query_point_list[i].u, query_point_list[i].v) << std::endl;
        }
      }
      std::sort(accesses.begin(), accesses.end());
      min_access_cached[d] = accesses.front();
      max_access_cached[d] = accesses.back();
      sum = std::accumulate(accesses.begin(), accesses.end(), 0.0);
      mean = sum / accesses.size();
      sq_sum = std::inner_product(accesses.begin(), accesses.end(), accesses.begin(), 0.0);
      stdev = std::sqrt(sq_sum / accesses.size() - mean * mean);
      avg_access_cached[d] = mean;
      std_access_cached[d] = stdev;
    }

    // Show the test results
    std::cout << "==== TIME ANALYSIS ====\n";
    for (int depth = 0; depth <= max_tree_depth; ++depth)
    {
      std::cout
          << " d[" << depth << "] - "
          << "build: " << time_construct[depth] << ", "
          << "insert: " << time_insert[depth] << ", "
          << "NN search (normal/cached): " << time_normal[depth] << ", " << time_cached[depth] << "\n";
    }
    std::cout << "==== ACCESS ANALYSIS ====\n";
    for (int depth = 0; depth < max_tree_depth; ++depth)
    {
      std::cout
          << " d[" << depth << "] - (normal/cached)"
          << "total: " << access_normal[depth] << ", " << access_cached[depth] << " / "
          << "min: " << min_access_normal[depth] << "," << min_access_cached[depth] << " / "
          << "max: " << max_access_normal[depth] << "," << max_access_cached[depth] << " / "
          << "avg: " << avg_access_normal[depth] << "," << avg_access_cached[depth] << " / "
          << "std: " << std_access_normal[depth] << "," << std_access_cached[depth] << std::endl;
    }
  }
  catch (std::exception &e)
  {
    std::cout << "EXCEPTION: " << e.what() << std::endl;
  }

  return 0;
}