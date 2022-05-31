#ifndef _QUADTREEFASTMULTIPLEPOOLED_H_
#define _QUADTREEFASTMULTIPLEPOOLED_H_

#include <iostream>
#include <vector>
#include <memory>
#include "QuadTreeFastPooled.h"

class QuadTreeFastMultiplePooled {
public:
	QuadTreeFastPooled* trees[8]; // �� 8���� tree�� �������ִ�. ���� �����ϵ��� ����.
	double n_cols;
	double n_rows;
public:
	QuadTreeFastMultiplePooled(); // �⺻������. �Ⱦ��ϰž�.
	QuadTreeFastMultiplePooled(std::vector<Point2<double>>& points_input_, 
		std::vector<int>& dirs1_input_,
		std::vector<int>& dirs2_input_,
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);
	
	~QuadTreeFastMultiplePooled();// �Ҹ���. ��� �޸� �Ѽ���.

	// cached node ���� �׳� ��Ī.
	void searchNN(
	std::vector<Point2<double>>& points_query,
	std::vector<int>& dirs,
	std::vector<int>& id_matched,
	std::vector<Node*>& nodes_matched);


	// cached node �̿��ؼ� ��Ī.
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
	// �ƹ��͵� ���ϴ� ������.
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
	// Ʈ������ �ʱ�ȭ�Ѵ�.
	// multiple�� ���� �߰����� �����ڸ� ���� ���� �������� ���·� �ʱ�ȭ�Ѵ�.
	for (int i = 0; i < 8; i++){
		trees[i] = 
		new QuadTreeFastPooled(n_rows_, n_cols_, max_depth_, eps_, dist_thres_, objpool_node_, objpool_elem_);
	}
	
	n_cols = n_cols_;
	n_rows = n_rows_;

	// �� direction�� ����� ������ �ش�Ǵ� tree�� �־��ش�.
	int dir1 = -1;
	int dir2 = -1;
	int counters[8] = { 0 }; // ���� �ش� tree�� ��������, 1�� ���Ѵ�. (id�� �ȴ�)
	for (int i = 0; i < points_input_.size(); i++) {
		// �ٶ�ǵ�, dir1�� ������ 0~7 �����ϰ��̴�.
		dir1 = dirs1_input_[i];
		dir2 = dirs2_input_[i];

		trees[dir1]->insertPublic(points_input_[i], counters[dir1], i);
		++counters[dir1];

		// dir2�� �ִ��� Ȯ���غ���.
		if (dir2 > -1) { // dir2�� �ֵ���, ����ε� points_input_�� �־�����.
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
	// �� direction�� ����� ������ �ش�Ǵ� tree���� �˻��Ѵ�.
	int dir = -1;
	int id_matched_temp = -1;
	for (int i = 0; i < points_query.size(); i++) {
		// �̹��� ���ο� �ִ� ���, ��Ī. 
		dir = dirs[i];
		if(points_query[i].u > 0 && points_query[i].u < n_cols && 
		   points_query[i].v > 0 && points_query[i].v < n_rows) {
			
			// printf("%d - th match... dir :%d\n",i, dir);
			// �ش� ���⿡�� ��Ī�� �Ѵ�.
			id_matched_temp = trees[dir]->searchNNSingleQuery(points_query[i], nodes_matched[i]);
			id_matched[i]=(id_matched_temp);
		}
		else{ // �̹��� ���ο� ���� ������, id_matched = -1, nodes_matched = root
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
	// �� direction�� ����� ������ �ش�Ǵ� tree���� �˻��Ѵ�.
	int dir = -1;
	int id_matched_temp = -1;
	for (int i = 0; i < points_query.size(); i++) {// �̹��� ���ο� �ִ� ���, ��Ī. 
		dir = dirs[i];	
		if(points_query[i].u > 0 && points_query[i].u < n_cols && 
		   points_query[i].v > 0 && points_query[i].v < n_rows) {

			// �ش� ���⿡�� ��Ī�� �Ѵ�.
			id_matched_temp = trees[dir]->searchNNSingleQueryCached(points_query[i], nodes_cached_matched[i]);
			id_matched[i]=(id_matched_temp);
		}
		else{ // �̹��� ���ο� ���� ������, id_matched = -1, nodes_matched = root
			id_matched[i]=(-2);
			nodes_cached_matched[i] = trees[dir]->root;
		}
	}
};
#endif