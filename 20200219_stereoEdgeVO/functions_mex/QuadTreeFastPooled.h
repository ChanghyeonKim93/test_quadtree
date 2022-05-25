#ifndef _QUADTREEFASTPOOLED_H_
#define _QUADTREEFASTPOOLED_H_

#define MAX_POOL 131072 // object pool의 최대 원소갯수.

#include <iostream>
#include <vector>
#include <memory>
#include "CommonStruct.h"
#include "CommonFunction.h"
#include "ObjectPool.h"

// Point2, Bound, Node, NodeStack, ElemOverlap, Elem : CommonStruct에 있다.
// Node와 Elem에 대해서 object pooling을 수행한다.
// 상속 모호성을 없애기 위해, 각  function으로 접근 할 때 한정자를 제대로 써줘야한다.
class QuadTreeFastPooled
{
public:
	// root node이다.
	Node* root;
private:
	// 두가지 objectpool이다.
	ObjectPool<Node>* objpool_node;
	ObjectPool<Elem>* objpool_elem;


	// root node 에서는 전체 rectangular size를 가지고있다. 
	// 어차피 65536 보다 큰 사이즈의 이미지를 쓰지 않으므로, ushort를 쓴다.
	ushort width;
	ushort height;

	// 최대 깊이.
	ushort MAX_DEPTH;
	double eps; // approximated quadtree에 쓰이는 epsilon 마진. 허용오차.
	double scale; // 1-eps 이다.
	double scale2; // (1-eps)^2 -> BOB랑 BWB에 쓰면 되나 ?

				   // 자주 쓰이는 변수. 선할당 해놓고 쓰자! 
	double* min_dist;
	double* dist_temp;
	double* dist_thres;

	// point들을 담고있는 vector이다. point 갯수만큼 길이를 가진다.
	std::vector<Elem*> elem_vector;
	std::vector<Node*> node_vector; // 모든 노드들을 담고있는 vector이다.

	size_t n_elem;// point 갯수랑 동일하다.
public:
	// recursive searching을 위한 Nodestack
	PointerStack<Node> stack;

	// private methods
private:
	// insert와 관련된 함수들
	//bool inBoundary();
	// 포인터의 레퍼런스로 하면, 포인터 8바이트의 복사가 일어나지 않는다.
	void selectChild(Node*& nd_, Elem*& elem_, int& flag_ew, int& flag_sn);
	void selectChild(Node*& nd_, Point2<double>& pt_, int& flag_ew, int& flag_sn); // overload

	void calcQuadrantRect(Node*& parent_, Node*& child_, int& flag_ew, int& flag_sn);
	void insert(const int& depth_, Elem* elem_q_, Node* nd_); // non-recursive version. (DFS에 대한 stack 이용)

	// search와 관련된 함수들
	void findNearestElem(Node*& nd_, Elem*& elem_q_, int& id_matched, Node*& nd_matched_);
	void findNearestElem(Node*& nd_, Point2<double>& pt_q_, int& id_matched, Node*& nd_matched_);
	bool BWBTest(Node*& nd_, Elem*& elem_q_);// Ball within bound
	bool BWBTest(Node*& nd_, Point2<double>& pt_q_);// Ball within bound
	bool BOBTest(Node*& nd_, Elem*& elem_q_);// Ball Overlap bound
	bool BOBTest(Node*& nd_, Point2<double>& pt_q_);// Ball Overlap bound
	bool inBound(Node*& nd_, Elem*& elem_q_); // in boundary test
	bool inBound(Node*& nd_, Point2<double>& pt_q_);

public:
	// for multiple,
	void insertPublic(Point2<double>& pt_, int& id_, int& id_pts_);

public:
	int searchNNSingleQuery(Point2<double>& pt_q_, Node*& node_matched_); // Stack version. non-recursion.
	int searchNNSingleQueryCached(Point2<double>& pt_q_, Node*& node_cached_and_matched_);

	void searchNNMultipleQueries(std::vector<Point2<double>>& pts, std::vector<int>& index_matched, std::vector<Node*>& nodes_matched);
	void searchNNMultipleQueriesCached(std::vector<Point2<double>>& pts, std::vector<int>& index_matched, std::vector<Node*>& nodes_cached_and_matched);

	// public methods
public:
	// 생성자
	QuadTreeFastPooled();
	QuadTreeFastPooled(std::vector<Point2<double>>& points_input_,
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);
	QuadTreeFastPooled( // pts 없이, 나머지 모두 초기화 하는 버전. (multiple 위해)
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);

	// 소멸자. 모든 메모리 뿌수자.
	~QuadTreeFastPooled();

	// 모든 노드 정보를 보여준다.
	void showAllNodes();

};

/*
* ----------------------------------------------------------------------------
*                                IMPLEMENTATION
* ----------------------------------------------------------------------------
*/
QuadTreeFastPooled::QuadTreeFastPooled() {
	this->root = nullptr;
	this->min_dist = new double(1e16);
	this->dist_temp = new double(0);
	printf("QuadtreePooled fast initiates.\n");
};

QuadTreeFastPooled::~QuadTreeFastPooled() {

	delete min_dist;
	delete dist_temp;
	delete dist_thres;
	for (int i = 0; i < node_vector.size(); i++) {
		objpool_node->returnObject(node_vector[i]);
	}
	for (int i = 0; i < elem_vector.size(); i++) {
		elem_vector[i]->next = nullptr;
		elem_vector[i]->id = -1;
		objpool_elem->returnObject(elem_vector[i]);
	}
};


QuadTreeFastPooled::QuadTreeFastPooled(
	int n_rows_, int n_cols_,
	int max_depth_,
	double eps_,
	double dist_thres_,
	ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_)
{
	objpool_node = objpool_node_;
	objpool_elem = objpool_elem_;

	// 우선, root부터 초기화 한다.
	// root = new Node(nullptr, 0, false); // root는 당연히~ parent가 없다.
	root = objpool_node->getObject();

	root->parent = nullptr;
	root->first_child = nullptr;
	root->isvisited = false;
	root->isleaf = false; // root는 시작될때 당연히 유일한 leaf이다.
	root->depth = 0; // 깊이는 0이다.
	root->bound.nw = Point2<ushort>(0, 0); // root의 바운더리 설정.
	root->bound.se = Point2<ushort>(n_cols_, n_rows_);

	// parent, first_child메모리 초기화, isleaf=0, header_elem = -1;
	// root->first_child = (Node*)malloc(sizeof(Node) * 4);
	node_vector.push_back(root);

	root->first_child = objpool_node->getObjectQuadruple();
	node_vector.push_back(root->first_child);
	node_vector.push_back(root->first_child + 1);
	node_vector.push_back(root->first_child + 2);
	node_vector.push_back(root->first_child + 3);

	root->initializeChildren();

	// 파라미터 초기화 
	// 최대 깊이.
	MAX_DEPTH = max_depth_;

	// 이미지 사이즈
	width = n_cols_;
	height = n_rows_;

	// 허용 approximate params
	eps = eps_;
	scale = 1.0 - eps;
	scale2 = scale*scale;

	// 자주쓰는 동적 할당 변수 정의.
	dist_thres = new double(dist_thres_*dist_thres_); // 제곱 거리다.
	dist_temp = new double(0);
	min_dist = new double(1e15);

};

QuadTreeFastPooled::QuadTreeFastPooled(
	std::vector<Point2<double>>& points_input_,
	int n_rows_, int n_cols_,
	int max_depth_,
	double eps_,
	double dist_thres_,
	ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_)
{
	objpool_node = objpool_node_;
	objpool_elem = objpool_elem_;

	// 우선, root부터 초기화 한다.
	// root = new Node(nullptr, 0, false); // root는 당연히~ parent가 없다.
	root = objpool_node->getObject();

	root->parent = nullptr;
	root->first_child = nullptr;
	root->isvisited = false; 
	root->isleaf = false; // root는 시작될때 당연히 유일한 leaf이다.
	root->depth = 0; // 깊이는 0이다.
	root->bound.nw = Point2<ushort>(0, 0); // root의 바운더리 설정.
	root->bound.se = Point2<ushort>(n_cols_, n_rows_);

	// parent, first_child메모리 초기화, isleaf=0, header_elem = -1;
	// root->first_child = (Node*)malloc(sizeof(Node) * 4);
	node_vector.push_back(root);

	root->first_child = objpool_node->getObjectQuadruple();
	node_vector.push_back(root->first_child);
	node_vector.push_back(root->first_child + 1);
	node_vector.push_back(root->first_child + 2);
	node_vector.push_back(root->first_child + 3);

	root->initializeChildren();

	// 파라미터 초기화 
	// 최대 깊이.
	MAX_DEPTH = max_depth_;

	// 이미지 사이즈
	width = n_cols_;
	height = n_rows_;

	// 허용 approximate params
	eps = eps_;
	scale = 1.0 - eps;
	scale2 = scale*scale;

	// 자주쓰는 동적 할당 변수 정의.
	dist_thres = new double(dist_thres_*dist_thres_); // 제곱 거리다.
	dist_temp = new double(0);
	min_dist = new double(1e15);

	// elem 초기화
	n_elem = points_input_.size();
	Elem* elem_temp = nullptr;
	for (int i = 0; i < n_elem; i++) {
		// = new Elem(points_input_[i]);
		elem_temp = objpool_elem->getObject();
		elem_temp->pt = points_input_[i];
		elem_temp->id = i;
		elem_temp->id_pts = i; // single case 에서는 id와 id_pts 가 같다.
		elem_temp->next = nullptr;
		elem_vector.push_back(elem_temp);

		// insert this element.
		insert(0, elem_temp, root);
	}

};

void QuadTreeFastPooled::selectChild(Node*& nd_, Elem*& elem_, int& flag_ew, int& flag_sn) {
	//Point<ushort> center;
	//center.u = ((nd_->bound.nw.u + nd_->bound.se.u) >> 1);
	//center.v = ((nd_->bound.nw.v + nd_->bound.se.v) >> 1);
	flag_ew = (nd_->bound.nw.u + nd_->bound.se.u) < elem_->pt.u * 2 ? 1 : 0; // 동쪽이면 1
	flag_sn = (nd_->bound.nw.v + nd_->bound.se.v) < elem_->pt.v * 2 ? 1 : 0; // 남쪽이면 1
};
void QuadTreeFastPooled::selectChild(Node*& nd_, Point2<double>& pt_, int& flag_ew, int& flag_sn) {
	//Point<ushort> center;
	//center.u = ((nd_->bound.nw.u + nd_->bound.se.u) >> 1);
	//center.v = ((nd_->bound.nw.v + nd_->bound.se.v) >> 1);
	flag_ew = (nd_->bound.nw.u + nd_->bound.se.u) < pt_.u * 2 ? 1 : 0; // 동쪽이면 1
	flag_sn = (nd_->bound.nw.v + nd_->bound.se.v) < pt_.v * 2 ? 1 : 0; // 남쪽이면 1
};

void QuadTreeFastPooled::calcQuadrantRect(
	Node*& parent_, Node*& child_, int& flag_ew, int& flag_sn)
{
	// 사분면 별 중심점과 우측 변 길이, 하단 변 길이를 저장.
	Point2<ushort> center;
	center.u = ((parent_->bound.nw.u + parent_->bound.se.u) >> 1);
	center.v = ((parent_->bound.nw.v + parent_->bound.se.v) >> 1);

	if (flag_ew) { // 동쪽이면,
		child_->bound.nw.u = center.u;
		child_->bound.se.u = parent_->bound.se.u;
	}
	else { // 서쪽
		child_->bound.nw.u = parent_->bound.nw.u;
		child_->bound.se.u = center.u;
	}

	if (flag_sn) { // 남쪽이면,
		child_->bound.nw.v = center.v;
		child_->bound.se.v = parent_->bound.se.v;
	}
	else { // 북쪽이면,
		child_->bound.nw.v = parent_->bound.nw.v;
		child_->bound.se.v = center.v;
	}
};

// ==================== 설명 ====================
// - 기본적으로, 들어오는 elem_q_->next = NULL이어야 한다.
// - nd_를 reference 로 전달 할 방법을 찾아보자.
// - child가 하나라도 생겨야 하는 상황이면, 한꺼번에 4개를 연속으로 할당하자.
// non-max depth -> bridge or leaf. (bridge는 다 똑같고, leaf는 nonmax leaf)
// max depth     -> leaf.
//
//                     [쿼리 별 ]
//                     [판단트리]
//                       /    \
//                      /      \
//                     /        \
//         [최대 깊이 O]         [최대 깊이 X]
//          무조건 leaf          leaf or bridge
//          /  \                            /   \
//         /    \                          /     \
//  [active O]   [active X]         [leaf]        [bridge]
//   append    activate+append    bridge로 변경,   그냥 자식으로 간다.
//                                여기 있던 점과
//                                함께 자식으로.
//
//      | 00 | 01 |   | HL | HR |
//      |----|----| = |----|----| : LSB(좌우), MSB(상하)
//      | 10 | 11 |   | BL | BR |
//
// [Leaf 특성]
// - 원소 존재 O (id_elem  > -1)
// [bridge 특성]
// - 원소 존재 X (id_elem == -1)

// [노드가 생성 될 때]
// - 새 노드가 생성 될때는 항상 leaf이기 때문에 생성되는 것임을 명심하자.
// - leaf가 생성 될 때는, 
//   -> isleaf = 1, id_elem = #, center와 half를 절반때려서 넣어줘야함.
//
// [leaf -> bridge]
//   -> node에 들어있던 elem + 새로 온 elem을 현재 node의 자식노드로 travel 해야한다.
//   -> isleaf = 0, id_elem = -1.

 // 이것도 사실 stack기반으로 만들 수 있을 것 같은데 ...
void QuadTreeFastPooled::insert(const int& depth_, Elem* elem_q_, Node* nd_) {
	// 1) 최대 깊이가 아닌 경우.
	if (depth_ < MAX_DEPTH) {
		// leaf 혹은 bridge이다.
		if (nd_->isleaf) {
			// leaf인 경우 -> bridge로 바뀌고, 
			// 기존점+쿼리점 둘 다 각각 원하는 곳의 자식쪽으로 간다.
			// 1. isleaf = false. 어차피 bridge될거임.
			nd_->isleaf = false; // bridge 설정 완료.

								 //2 . 지금 node에 있던 elem의 포인터를 뽑아오고, node의 header는 -1로 바꿈.
			Elem* elem_temp = this->elem_vector[nd_->header_elem];
			nd_->header_elem = -1;

			// TODO: memorypool로 바꾸자.
			// 3. children의 공간을 할당 (및 일부초기화)를 수행한다. 
			// parent, first_child메모리 초기화, isleaf=0,  header_elem = -1;
			//nd_->first_child = (Node*)malloc(sizeof(Node) * 4);
			nd_->first_child = objpool_node->getObjectQuadruple();
			node_vector.push_back(nd_->first_child);
			node_vector.push_back(nd_->first_child + 1);
			node_vector.push_back(nd_->first_child + 2);
			node_vector.push_back(nd_->first_child + 3);

			nd_->initializeChildren();

			// 4. elem_temp을 travel.
			int flag_ew, flag_sn, num_child; // 동쪽이면 flag_ew: 1, 남쪽이면 flag_sn: 1
			selectChild(nd_, elem_temp, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			Node* child = (nd_->first_child + num_child); // 선택된 자식 방향.

														  // 원래 bridge였으므로, 자식 체크 필요 X. 걍 leaf로 만들어준다.
			child->isleaf = true;
			child->depth = depth_ + 1;
			// 자식 노드가 된 녀석의 모서리 값을 계산해준다.
			calcQuadrantRect(nd_, child, flag_ew, flag_sn);
			// header_elem의 값을 지금의 id로 주고, 현재 노드의 element 1개로 늘린다.
			child->header_elem = elem_temp->id;


			// 5. elem_q_을 travel. 여기서는 원하는곳에 자식이 있는지 없는지 확인해야한다.
			selectChild(nd_, elem_q_, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			child = (nd_->first_child + num_child);
			// 자식이 leaf인지, 아니면 아무 할당이 되어있지 않은지 테스트해야함.
			if (!child->isleaf) {
				child->isleaf = true; // leaf로 만들어줌.
				child->depth = depth_ + 1;
				// 자식 노드가 된 녀석의 모서리 값을 계산해준다.
				calcQuadrantRect(nd_, child, flag_ew, flag_sn);
				child->header_elem = elem_q_->id;
			}

			// child node가 채워져 있을때는 그냥 아래로 ㄱㄱ
			else insert(depth_ + 1, elem_q_, child);
		}
		else {
			// bridge인 경우 -> 바로 자식으로 가면 된다. 
			// 원하는 곳에 자식이 있는 경우 / 없는 경우로 나뉨.
			int flag_ew, flag_sn, num_child; // 동쪽이면 flag_ew: 1, 남쪽이면 flag_sn: 1
			selectChild(nd_, elem_q_, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			Node* child = nd_->first_child + num_child;

			// 자식이( bridge 또는 leaf인지), 아니면 아무 할당이 되어있지 않은지 테스트해야함.
			// 해당 자식노드가 초기화되어있지 않았다면, depth = 0이다. depth =0은 root빼고 없다.
			if (!child->depth) {
				child->isleaf = true; // leaf로 만들어줌.
				child->depth = depth_ + 1;
				// 자식 노드가 된 녀석의 모서리 값을 계산해준다.
				calcQuadrantRect(nd_, child, flag_ew, flag_sn);
				child->header_elem = elem_q_->id;
			}

			// child node가 채워져 있을때는 그냥 아래로 ㄱㄱ
			else insert(depth_ + 1, elem_q_, child);
		}
	}
	// 2) 최대 깊이인 경우에는 무조건 1개 이상의 원소를 가진 leaf이기 때문에, 그냥 붙이면된다.
	else {
		// 지금 leaf의 header이다.
		// 다음 것이 없다면,현재 leaf의 elem중 마지막 것을 찾는다.
		Elem* elem_ptr = this->elem_vector[nd_->header_elem]; 
		while (elem_ptr->next) elem_ptr = elem_ptr->next; 
		elem_ptr->next = elem_q_; // next랑 elem_q_랑 연결되었다!
	}
};

void QuadTreeFastPooled::insertPublic(Point2<double>& pt_, int& id_, int& id_pts_) {
	Elem* elem_temp = objpool_elem->getObject();
	elem_temp->pt = pt_;
	elem_temp->id = id_; // 내부적으로 매기는 elem의 id임.
	elem_temp->id_pts = id_pts_; // 외부적으로 매겨진 id_pts.
	elem_temp->next = nullptr;
	elem_vector.push_back(elem_temp);

	// insert this element.
	insert(0, elem_temp, root);
};

void QuadTreeFastPooled::findNearestElem(Node*& nd_, Elem*& elem_q_, int& id_matched, Node*& nd_matched_) {
	Elem* elem_ptr = this->elem_vector[nd_->header_elem]; // 지금 leaf의 header이다.
														  // 현재 점과의 거리를 구하고, 다음 점으로 넘겨준다. 다음 점이 nullptr이면 종료.
	while (elem_ptr) {
		*this->dist_temp = distEuclidean(elem_ptr->pt, elem_q_->pt);
		if (*this->min_dist > *this->dist_temp) {
			*this->min_dist = *this->dist_temp;
			id_matched  = elem_ptr->id_pts;
			nd_matched_ = nd_;
		}
		elem_ptr = elem_ptr->next;
	}
};
void QuadTreeFastPooled::findNearestElem(Node*& nd_, Point2<double>& pt_q_, int& id_matched, Node*& nd_matched_) {
	Elem* elem_ptr = this->elem_vector[nd_->header_elem]; // 지금 leaf의 header이다.
														  // 현재 점과의 거리를 구하고, 다음 점으로 넘겨준다. 다음 점이 nullptr이면 종료.
	while (elem_ptr) {
		*this->dist_temp = distEuclidean(elem_ptr->pt, pt_q_);
		if (*this->min_dist > *this->dist_temp) {
			*this->min_dist = *this->dist_temp;
			id_matched = elem_ptr->id_pts;
			nd_matched_ = nd_;
		}
		elem_ptr = elem_ptr->next;
	}
};

// node 내부에 query 점과 현재 min_dist로 이루어진 원이 들어오는지?
bool QuadTreeFastPooled::BWBTest(Node*& nd_, Elem*& elem_q_) {
	if (nd_->parent == nullptr) return true;
	double d_hori, d_vert;
	if (nd_->bound.nw.u == 0)
		d_hori = nd_->bound.se.u - elem_q_->pt.u;
	else if (nd_->bound.se.u == this->width)
		d_hori = elem_q_->pt.u - nd_->bound.nw.u;
	else
		d_hori = nd_->bound.se.u - elem_q_->pt.u < elem_q_->pt.u - nd_->bound.nw.u ? nd_->bound.se.u - elem_q_->pt.u : elem_q_->pt.u - nd_->bound.nw.u;

	if (nd_->bound.nw.v == 0)
		d_vert = nd_->bound.se.v - elem_q_->pt.v;
	else if (nd_->bound.se.v == this->height)
		d_vert = elem_q_->pt.v - nd_->bound.nw.v;
	else
		d_vert = nd_->bound.se.v - elem_q_->pt.v < elem_q_->pt.v - nd_->bound.nw.v ? nd_->bound.se.v - elem_q_->pt.v : elem_q_->pt.v - nd_->bound.nw.v;

	double d_min = d_hori < d_vert ? d_hori : d_vert;
	return (*this->min_dist*scale2 < d_min*d_min);
};
bool QuadTreeFastPooled::BWBTest(Node*& nd_, Point2<double>& pt_q_) {
	/*double d_a = pt_q_.v - (double)nd_->bound.nw.v;
	double d_b = (double)nd_->bound.se.v - pt_q_.v;
	double d_vert = d_a < d_b ? d_a : d_b;

	double d_c = pt_q_.u - (double)nd_->bound.nw.u;
	double d_d = (double)nd_->bound.se.u - pt_q_.u;
	double d_hori = d_c < d_d ? d_c : d_d;*/
	if (nd_->parent == nullptr) return true;

	double d_hori, d_vert;
	if (nd_->bound.nw.u == 0)
		d_hori = (double)nd_->bound.se.u - pt_q_.u;
	else if (nd_->bound.se.u == this->width)
		d_hori = pt_q_.u - (double)nd_->bound.nw.u;
	else
		d_hori = (double)nd_->bound.se.u - pt_q_.u < pt_q_.u - (double)nd_->bound.nw.u
		? (double)nd_->bound.se.u - pt_q_.u : pt_q_.u - (double)nd_->bound.nw.u;

	if (nd_->bound.nw.v == 0)
		d_vert = nd_->bound.se.v - pt_q_.v;
	else if (nd_->bound.se.v == this->height)
		d_vert = pt_q_.v - nd_->bound.nw.v;
	else
		d_vert = (double)nd_->bound.se.v - pt_q_.v < pt_q_.v - (double)nd_->bound.nw.v
		? (double)nd_->bound.se.v - pt_q_.v : pt_q_.v - (double)nd_->bound.nw.v;

	double d_min = d_hori < d_vert ? d_hori : d_vert;
	// std::cout << "a,b,c,d: " << d_a << ", " << d_b << ", " << d_c << ", " << d_d << std::endl;
	return (*this->min_dist*scale2 < d_min*d_min);
};

// node 내부에 점이 존재하는지 확인.
bool QuadTreeFastPooled::inBound(Node*& nd_, Elem*& elem_q_) {
	return ((nd_->bound.nw.u < elem_q_->pt.u)
		&& (elem_q_->pt.u < nd_->bound.se.u)
		&& (nd_->bound.nw.v < elem_q_->pt.v)
		&& (elem_q_->pt.v < nd_->bound.se.v));
};
bool QuadTreeFastPooled::inBound(Node*& nd_, Point2<double>& pt_q_) {
	return ((nd_->bound.nw.u < pt_q_.u)
		&& (pt_q_.u < nd_->bound.se.u)
		&& (nd_->bound.nw.v < pt_q_.v)
		&& (pt_q_.v < nd_->bound.se.v));
};

bool QuadTreeFastPooled::BOBTest(Node*& nd_, Elem*& elem_q_) {
	// 좌
	double min_dist_scale = *this->min_dist*scale2;
	if (elem_q_->pt.u < nd_->bound.nw.u)
		// 좌상
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u)
			+ (elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// 좌중
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u);
	// 좌하
		else
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u)
			+ (elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
	// 중
	else if (elem_q_->pt.u < nd_->bound.se.u)
		// 중상
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// 중중은 없다.
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return true; // 무조건 겹친다.
						 // 중하
		else
			return min_dist_scale >
			(elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
	// 우
	else
		// 우상
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u)
			+ (elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// 우중
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u);
	// 우하
		else
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u)
			+ (elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
};
bool QuadTreeFastPooled::BOBTest(Node*& nd_, Point2<double>& pt_q_) {
	// 좌
	double min_dist_scale = *this->min_dist*scale2;
	if (pt_q_.u < nd_->bound.nw.u)
		// 좌상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 좌중
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u);
	// 좌하
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// 중
	else if (pt_q_.u < nd_->bound.se.u)
		// 중상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 중중은 없다.
		else if (pt_q_.v < nd_->bound.se.v)
			return true;
	// 중하
		else
			return min_dist_scale >
			(pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// 우
	else
		// 우상
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// 우중
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u);
	// 우하
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
};


int QuadTreeFastPooled::searchNNSingleQuery(Point2<double>& pt_q_, Node*& node_matched_) {
	// root를 stack 처음에 넣는다.
	stack.push(this->root);

	// 거리 변수 초기화
	*this->min_dist = 1e15;
	*this->dist_temp = 0;
	Node* node = nullptr;
	Node* child = nullptr;
	int flag_ew, flag_sn;
	int id_elem_matched = -1;
	node_matched_ = nullptr;

	while (stack.size > 0) {
		node = stack.top();
		stack.pop();
		// leaf node이면 가장 가까운 점을 찾는다.
		if (node->isleaf) {
			stack.total_access++;
			findNearestElem(node, pt_q_, id_elem_matched, node_matched_);
			if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) break;
		}
		// leaf node가 아니면, 어디를 가야 할 지 결정한다.
		else {
			// 현재의 node가 BOB 만족하지 않으면, 자식 노드로 갈 필요가 없다.
			if (BOBTest(node, pt_q_)) {
				stack.total_access++;
				selectChild(node, pt_q_, flag_ew, flag_sn);

				child = node->first_child + (!flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (!flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				// 지금 child.
				child = node->first_child + (flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
			}
		}
	}

	stack.clear(); // stack 초기화. (메모리 내용만 초기화)
	return id_elem_matched;
};

int QuadTreeFastPooled::searchNNSingleQueryCached(Point2<double>& pt_q_, Node*& node_cached_and_matched_) {
	// 0) 거리 변수 초기화
	*this->min_dist = 1e15;
	*this->dist_temp = 0;
	stack.total_access++; // node 방문했으니, counter 올림.
	int id_elem_matched = -1; // 매칭된 점의 index
	Node* node = node_cached_and_matched_; // 우선 cached node를 불러온다.

	// root가 들어왔으면 걍 single.
	if(node == this->root){
		id_elem_matched = searchNNSingleQuery(pt_q_, node_cached_and_matched_);
		goto flagFinish;
	}
	node_cached_and_matched_ = nullptr;  // 매칭된 점이 존재하는 leaf node의 포인터.
										 // 1) nd_cached_내의 점에서 q와 가장 가까운 점을 찾는다. (무조건 leaf여야 한다...)
										 // 만일, q가 cached node의 bound 안에 위치하고, BWB까지 만족하면, 그냥 끝!
										 // 만약, inbound 인데, BWB만 만족하지 않았던 것이라면, parent로 갔다가 다시 여기로 내려올듯...
	findNearestElem(node, pt_q_, id_elem_matched, node_cached_and_matched_);

	if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) {
		goto flagFinish;
	}

	// 2) BWBTest == true 인 bridge까지 올라간다.
	node = node->parent;
	while (true) {
		//if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) break;
		if (inBound(node, pt_q_)) break; // PWB Test only.

		if (node->parent == nullptr) break; // root
		node = node->parent;
		stack.total_access++;
	}

	// 3) 현재 위치한 node부터 아래로 정방향 search 시작.
	stack.push(node);
	// 거리 변수 초기화
	int flag_ew, flag_sn;
	Node* child = nullptr;
	while (stack.size > 0) {
		node = stack.top();
		stack.pop();
		// leaf node이면 가장 가까운 점을 찾는다.
		if (node->isleaf) {
			stack.total_access++;
			findNearestElem(node, pt_q_, id_elem_matched, node_cached_and_matched_);
			// if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) goto Finish;
			if (inBound(node, pt_q_)) goto flagFinish; // PWB Test only.

		}
		// leaf node가 아니면, 어디를 가야 할 지 결정한다.
		else {
			// 현재의 node가 BOB 만족하지 않으면, 자식 노드로 갈 필요가 없다.
			// BOB를 만족하면, 자식 노드로 향한다.
			if (BOBTest(node, pt_q_)) {
				stack.total_access++;
				selectChild(node, pt_q_, flag_ew, flag_sn);
				// 지금 가려는 child가 아닌 children이 나중에 연산되어야 하니, 먼저 stack에 들어감.
				// 먼저 != nullptr && BOB pass 인 점들만 넣음.
				// 다른 child들, 초기화 된 놈은 depth가 무조건 >0 이다.
				// 초기화 되지 않은 놈은 0으로 되어있다. root 제외하고는 0을 가질수없다.
				child = node->first_child + (!flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (!flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				// 지금 child.
				child = node->first_child + (flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
			}
		}
	}

	// 스택을 다 사용하였으니, top을 0으로 만들어준다. (실제로 데이터를 지울 필요는 없다.)
flagFinish:
	stack.clear();
	return id_elem_matched;
};


void QuadTreeFastPooled::searchNNMultipleQueries(std::vector<Point2<double>>& pts, std::vector<int>& index_matched, std::vector<Node*>& nodes_matched) {
	int n_pts = pts.size();

	for (int i = 0; i < n_pts; i++) {
		if (inBound(this->root, pts[i])) { // 이미지 내부에 있으면 search
			index_matched[i] = searchNNSingleQuery(pts[i], nodes_matched[i]);
		}
		else { // 이미지 내부가 아니면, 기각.
			index_matched[i] = -2;
			nodes_matched[i] = this->root;
		}
	}
};

void QuadTreeFastPooled::searchNNMultipleQueriesCached(
	std::vector<Point2<double>>& pts,
	std::vector<int>& index_matched, std::vector<Node*>& nodes_cached_and_matched) {
	int n_pts = pts.size();
	for (int i = 0; i < n_pts; i++) {
		if (inBound(this->root, pts[i])) { // 이미지 내부에 있으면 search
			index_matched[i] = searchNNSingleQueryCached(pts[i], nodes_cached_and_matched[i]); // cached node는 입력.
		}
		else { // 이미지 내부가 아니면, 기각.
			index_matched[i] = -2;
			nodes_cached_and_matched[i] = this->root;
		}
	}
};

void QuadTreeFastPooled::showAllNodes() {
	// 첫번째는 무조건 root node이다.
	Node* node_temp = this->root;
	node_temp->showNodeSpec();
	for (int i = 1; i < node_vector.size(); i++) {
		node_temp = node_vector[i];
		if(node_temp->parent != nullptr)
			node_temp->showNodeSpec();
	}
};

#endif