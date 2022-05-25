#ifndef _QUADTREEFASTMULTIPLEPOOLED_H_
#define _QUADTREEFASTMULTIPLEPOOLED_H_

#include <iostream>
#include <vector>
#include <memory>
#include "QuadTreeFastPooled.h"

class QuadTreeFastMultiplePooled {
public:
	QuadTreeFastPooled* trees[8]; // 총 8개의 tree를 가지고있다. 각각 생성하도록 하자.
	double n_cols;
	double n_rows;
public:
	QuadTreeFastMultiplePooled(); // 기본생성자. 안쓰일거야.
	QuadTreeFastMultiplePooled(std::vector<Point2<double>>& points_input_, 
		std::vector<int>& dirs1_input_,
		std::vector<int>& dirs2_input_,
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);
	
	~QuadTreeFastMultiplePooled();// 소멸자. 모든 메모리 뿌수자.

	// cached node 없이 그냥 써칭.
	void searchNN(
	std::vector<Point2<double>>& points_query,
	std::vector<int>& dirs,
	std::vector<int>& id_matched,
	std::vector<Node*>& nodes_matched);


	// cached node 이용해서 써칭.
	void searchNNCached(
	std::vector<Point2<double>>& points_query,
	std::vector<int>& dirs,
	std::vector<int>& id_matched,
	std::vector<Node*>& nodes_cached_matched);
};

/*
* ----------------------------------------------------------------------------
*                                IMPLEMENTATION
* ----------------------------------------------------------------------------
*/
QuadTreeFastMultiplePooled::QuadTreeFastMultiplePooled() {
	for (int i = 0; i < 8; i++) {
		trees[i] = nullptr;
	};
	// 아무것도 안하는 생성자.
};

QuadTreeFastMultiplePooled::QuadTreeFastMultiplePooled(std::vector<Point2<double>>& points_input_,
	std::vector<int>& dirs1_input_,
	std::vector<int>& dirs2_input_,
	int n_rows_, int n_cols_,
	int max_depth_,
	double eps_,
	double dist_thres_,
	ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_)
{
	printf("start multiple trees generation...\n");
	// 트리들을 초기화한다.
	// multiple을 위한 추가적인 생성자를 통해 점을 넣지않은 상태로 초기화한다.
	for (int i = 0; i < 8; i++){
		trees[i] = 
		new QuadTreeFastPooled(n_rows_, n_cols_, max_depth_, eps_, dist_thres_, objpool_node_, objpool_elem_);
	}
	
	n_cols = n_cols_;
	n_rows = n_rows_;

	// 각 direction을 고려해 점들을 해당되는 tree로 넣어준다.
	int dir1 = -1;
	int dir2 = -1;
	int counters[8] = { 0 }; // 점이 해당 tree로 들어갈때마다, 1씩 더한다. (id가 된다)
	for (int i = 0; i < points_input_.size(); i++) {
		// 바라건데, dir1은 무조건 0~7 사이일것이다.
		dir1 = dirs1_input_[i];
		dir2 = dirs2_input_[i];

		trees[dir1]->insertPublic(points_input_[i], counters[dir1], i);
		++counters[dir1];

		// dir2가 있는지 확인해보자.
		if (dir2 > -1) { // dir2가 있따면, 여기로도 points_input_을 넣어주자.
			trees[dir2]->insertPublic(points_input_[i], counters[dir2], i);
			++counters[dir2];
		}
	}
	printf("Multiple tree made done.\n");
	printf("Point numbers:\n");
	for (int i = 0; i < 8; i++) {
		printf("dir [%d] - %d\n", i, counters[i]);
	}
	int sum_overlap = 0;
	for(int i = 0; i < 8; i++) sum_overlap += counters[i];
	printf("# pts (raw): %d / # pts (overlap): %d / increased: %d\n",
	points_input_.size(), sum_overlap, sum_overlap - points_input_.size());
	printf("\n\n");
};

QuadTreeFastMultiplePooled::~QuadTreeFastMultiplePooled() {
	for (int i = 0; i < 8; i++) delete trees[i];
};

void QuadTreeFastMultiplePooled::searchNN(
	std::vector<Point2<double>>& points_query,
	std::vector<int>& dirs,
	std::vector<int>& id_matched,
	std::vector<Node*>& nodes_matched)
{
	// 각 direction을 고려해 점들을 해당되는 tree에서 검색한다.
	int dir = -1;
	int id_matched_temp = -1;
	for (int i = 0; i < points_query.size(); i++) {
		// 이미지 내부에 있는 경우, 서칭. 
		dir = dirs[i];
		if(points_query[i].u > 0 && points_query[i].u < n_cols && 
		   points_query[i].v > 0 && points_query[i].v < n_rows) {
			
			// printf("%d - th match... dir :%d\n",i, dir);
			// 해당 방향에서 서칭을 한다.
			id_matched_temp = trees[dir]->searchNNSingleQuery(points_query[i], nodes_matched[i]);
			id_matched[i]=(id_matched_temp);
		}
		else{ // 이미지 내부에 있지 않으면, id_matched = -1, nodes_matched = root
			id_matched[i]=(-2);
			nodes_matched[i] = this->trees[dir]->root;
		}
		
	}
};

void QuadTreeFastMultiplePooled::searchNNCached(
	std::vector<Point2<double>>& points_query,
	std::vector<int>& dirs,
	std::vector<int>& id_matched,
	std::vector<Node*>& nodes_cached_matched)
{
	// 각 direction을 고려해 점들을 해당되는 tree에서 검색한다.
	int dir = -1;
	int id_matched_temp = -1;
	for (int i = 0; i < points_query.size(); i++) {// 이미지 내부에 있는 경우, 서칭. 
		dir = dirs[i];	
		if(points_query[i].u > 0 && points_query[i].u < n_cols && 
		   points_query[i].v > 0 && points_query[i].v < n_rows) {

			// 해당 방향에서 서칭을 한다.
			id_matched_temp = trees[dir]->searchNNSingleQueryCached(points_query[i], nodes_cached_matched[i]);
			id_matched[i]=(id_matched_temp);
		}
		else{ // 이미지 내부에 있지 않으면, id_matched = -1, nodes_matched = root
			id_matched[i]=(-2);
			nodes_cached_matched[i] = trees[dir]->root;
		}
	}
};
#endif