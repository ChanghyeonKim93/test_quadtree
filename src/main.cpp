#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <random>
#include <ctime>
#include <functional>
#include "Quadtree.h"

void print_start();

int main() {
  print_start();

  clock_t start, finish;
	std::mt19937 engine((unsigned int)time(NULL));
	std::uniform_real_distribution<> distribution(100.0, 500.0);
	auto generator = std::bind(distribution, engine);

	int numOfPoints  = 10000;
  int numOfQueries = 5000;
	int ndim         = 2;

	int binSize      = 30; // bin size : 10~50 (recommanded).
  double distThres = 150000;
	int maxDepth     = (int)ceil( log2(  ceil( (double)numOfPoints / (double)binSize ) ) );

  maxDepth = 6;

	std::vector<Point> points_vec;
	for (int i = 0; i < numOfPoints; i++) {
		points_vec.push_back(Point(generator(),generator(),i));
	}

 	std::vector<Point> points_q_vec;
  points_q_vec.reserve(numOfQueries);
  
  for (int i = 0; i < numOfQueries; i++){
    points_q_vec.push_back(Point(points_vec[i].u,points_vec[i].v,i));
  }


	// tree generation
  std::cout << "max depth : " << maxDepth << std::endl;

  start  = clock();
  QuadTree* tree = NULL;
  tree = new QuadTree(points_vec, 640,640, maxDepth, 0.1); // under 2 ms/5000, 6 ms/ 30000;
  finish = clock();
  std::cout << "QuadTree gen elapsed time : " << (finish - start) / 1000.0 << "[ms]"<< std::endl;
  
  start  = clock();
  std::vector<Node*> nodes_vec;
  nodes_vec.reserve(numOfPoints); 
  Point pt_temp;
  for(int i = 0; i < numOfQueries; i++){
    pt_temp = tree->cachedNNSearch(tree->root, points_q_vec[i]);
    nodes_vec.push_back((Node*)pt_temp.node_ptr);
  }
  finish = clock();
	std::cout << "quadtree first search elapsed time : " << (finish - start) / 1000.0 << "[ms]"<< std::endl;


  start  = clock();
  std::vector<int> indexVec;
  indexVec.reserve(numOfPoints);
  for(int i = 0; i < numOfQueries; i++){
     tree->cachedNNSearch((Node*)nodes_vec[i], points_q_vec[i]);
  }
  finish = clock();
	std::cout << "quadtree cached search elapsed time : " << (finish - start) / 1000.0 << "[ms]"<< std::endl;

  /*start = clock();
  for(int i =0; i<100; i++) kdtree->kdtree_nearest_neighbor(points_q_vec, indexVec);
  finish = clock();
	std::cout << "KD Search elapsed time : " << (finish - start) / 1000.0 << "[ms]"<< std::endl;

  //tree->print_tree(tree->treeRootNode,0);
	//tree->print_leaf(tree->treeRootNode,0);
*/
  delete tree;
	std::cout << "Program is running !" << std::endl;
	return 0;
}



/** 
 * 
 * functions
 * 
 */

void print_start()
{
  printf("\n----------------------------\n");
  printf("-                          -\n");
  printf("-      program starts.     -\n");
  printf("-                          -\n");
  printf("----------------------------\n\n\n");
}
