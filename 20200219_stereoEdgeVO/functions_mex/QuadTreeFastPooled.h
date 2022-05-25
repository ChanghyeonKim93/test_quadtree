#ifndef _QUADTREEFASTPOOLED_H_
#define _QUADTREEFASTPOOLED_H_

#define MAX_POOL 131072 // object pool�� �ִ� ���Ұ���.

#include <iostream>
#include <vector>
#include <memory>
#include "CommonStruct.h"
#include "CommonFunction.h"
#include "ObjectPool.h"

// Point2, Bound, Node, NodeStack, ElemOverlap, Elem : CommonStruct�� �ִ�.
// Node�� Elem�� ���ؼ� object pooling�� �����Ѵ�.
// ��� ��ȣ���� ���ֱ� ����, ��  function���� ���� �� �� �����ڸ� ����� ������Ѵ�.
class QuadTreeFastPooled
{
public:
	// root node�̴�.
	Node* root;
private:
	// �ΰ��� objectpool�̴�.
	ObjectPool<Node>* objpool_node;
	ObjectPool<Elem>* objpool_elem;


	// root node ������ ��ü rectangular size�� �������ִ�. 
	// ������ 65536 ���� ū �������� �̹����� ���� �����Ƿ�, ushort�� ����.
	ushort width;
	ushort height;

	// �ִ� ����.
	ushort MAX_DEPTH;
	double eps; // approximated quadtree�� ���̴� epsilon ����. ������.
	double scale; // 1-eps �̴�.
	double scale2; // (1-eps)^2 -> BOB�� BWB�� ���� �ǳ� ?

				   // ���� ���̴� ����. ���Ҵ� �س��� ����! 
	double* min_dist;
	double* dist_temp;
	double* dist_thres;

	// point���� ����ִ� vector�̴�. point ������ŭ ���̸� ������.
	std::vector<Elem*> elem_vector;
	std::vector<Node*> node_vector; // ��� ������ ����ִ� vector�̴�.

	size_t n_elem;// point ������ �����ϴ�.
public:
	// recursive searching�� ���� Nodestack
	PointerStack<Node> stack;

	// private methods
private:
	// insert�� ���õ� �Լ���
	//bool inBoundary();
	// �������� ���۷����� �ϸ�, ������ 8����Ʈ�� ���簡 �Ͼ�� �ʴ´�.
	void selectChild(Node*& nd_, Elem*& elem_, int& flag_ew, int& flag_sn);
	void selectChild(Node*& nd_, Point2<double>& pt_, int& flag_ew, int& flag_sn); // overload

	void calcQuadrantRect(Node*& parent_, Node*& child_, int& flag_ew, int& flag_sn);
	void insert(const int& depth_, Elem* elem_q_, Node* nd_); // non-recursive version. (DFS�� ���� stack �̿�)

	// search�� ���õ� �Լ���
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
	// ������
	QuadTreeFastPooled();
	QuadTreeFastPooled(std::vector<Point2<double>>& points_input_,
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);
	QuadTreeFastPooled( // pts ����, ������ ��� �ʱ�ȭ �ϴ� ����. (multiple ����)
		int n_rows_, int n_cols_,
		int max_depth_,
		double eps_,
		double dist_thres_,
		ObjectPool<Node>* objpool_node_, ObjectPool<Elem>* objpool_elem_);

	// �Ҹ���. ��� �޸� �Ѽ���.
	~QuadTreeFastPooled();

	// ��� ��� ������ �����ش�.
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

	// �켱, root���� �ʱ�ȭ �Ѵ�.
	// root = new Node(nullptr, 0, false); // root�� �翬��~ parent�� ����.
	root = objpool_node->getObject();

	root->parent = nullptr;
	root->first_child = nullptr;
	root->isvisited = false;
	root->isleaf = false; // root�� ���۵ɶ� �翬�� ������ leaf�̴�.
	root->depth = 0; // ���̴� 0�̴�.
	root->bound.nw = Point2<ushort>(0, 0); // root�� �ٿ���� ����.
	root->bound.se = Point2<ushort>(n_cols_, n_rows_);

	// parent, first_child�޸� �ʱ�ȭ, isleaf=0, header_elem = -1;
	// root->first_child = (Node*)malloc(sizeof(Node) * 4);
	node_vector.push_back(root);

	root->first_child = objpool_node->getObjectQuadruple();
	node_vector.push_back(root->first_child);
	node_vector.push_back(root->first_child + 1);
	node_vector.push_back(root->first_child + 2);
	node_vector.push_back(root->first_child + 3);

	root->initializeChildren();

	// �Ķ���� �ʱ�ȭ 
	// �ִ� ����.
	MAX_DEPTH = max_depth_;

	// �̹��� ������
	width = n_cols_;
	height = n_rows_;

	// ��� approximate params
	eps = eps_;
	scale = 1.0 - eps;
	scale2 = scale*scale;

	// ���־��� ���� �Ҵ� ���� ����.
	dist_thres = new double(dist_thres_*dist_thres_); // ���� �Ÿ���.
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

	// �켱, root���� �ʱ�ȭ �Ѵ�.
	// root = new Node(nullptr, 0, false); // root�� �翬��~ parent�� ����.
	root = objpool_node->getObject();

	root->parent = nullptr;
	root->first_child = nullptr;
	root->isvisited = false; 
	root->isleaf = false; // root�� ���۵ɶ� �翬�� ������ leaf�̴�.
	root->depth = 0; // ���̴� 0�̴�.
	root->bound.nw = Point2<ushort>(0, 0); // root�� �ٿ���� ����.
	root->bound.se = Point2<ushort>(n_cols_, n_rows_);

	// parent, first_child�޸� �ʱ�ȭ, isleaf=0, header_elem = -1;
	// root->first_child = (Node*)malloc(sizeof(Node) * 4);
	node_vector.push_back(root);

	root->first_child = objpool_node->getObjectQuadruple();
	node_vector.push_back(root->first_child);
	node_vector.push_back(root->first_child + 1);
	node_vector.push_back(root->first_child + 2);
	node_vector.push_back(root->first_child + 3);

	root->initializeChildren();

	// �Ķ���� �ʱ�ȭ 
	// �ִ� ����.
	MAX_DEPTH = max_depth_;

	// �̹��� ������
	width = n_cols_;
	height = n_rows_;

	// ��� approximate params
	eps = eps_;
	scale = 1.0 - eps;
	scale2 = scale*scale;

	// ���־��� ���� �Ҵ� ���� ����.
	dist_thres = new double(dist_thres_*dist_thres_); // ���� �Ÿ���.
	dist_temp = new double(0);
	min_dist = new double(1e15);

	// elem �ʱ�ȭ
	n_elem = points_input_.size();
	Elem* elem_temp = nullptr;
	for (int i = 0; i < n_elem; i++) {
		// = new Elem(points_input_[i]);
		elem_temp = objpool_elem->getObject();
		elem_temp->pt = points_input_[i];
		elem_temp->id = i;
		elem_temp->id_pts = i; // single case ������ id�� id_pts �� ����.
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
	flag_ew = (nd_->bound.nw.u + nd_->bound.se.u) < elem_->pt.u * 2 ? 1 : 0; // �����̸� 1
	flag_sn = (nd_->bound.nw.v + nd_->bound.se.v) < elem_->pt.v * 2 ? 1 : 0; // �����̸� 1
};
void QuadTreeFastPooled::selectChild(Node*& nd_, Point2<double>& pt_, int& flag_ew, int& flag_sn) {
	//Point<ushort> center;
	//center.u = ((nd_->bound.nw.u + nd_->bound.se.u) >> 1);
	//center.v = ((nd_->bound.nw.v + nd_->bound.se.v) >> 1);
	flag_ew = (nd_->bound.nw.u + nd_->bound.se.u) < pt_.u * 2 ? 1 : 0; // �����̸� 1
	flag_sn = (nd_->bound.nw.v + nd_->bound.se.v) < pt_.v * 2 ? 1 : 0; // �����̸� 1
};

void QuadTreeFastPooled::calcQuadrantRect(
	Node*& parent_, Node*& child_, int& flag_ew, int& flag_sn)
{
	// ��и� �� �߽����� ���� �� ����, �ϴ� �� ���̸� ����.
	Point2<ushort> center;
	center.u = ((parent_->bound.nw.u + parent_->bound.se.u) >> 1);
	center.v = ((parent_->bound.nw.v + parent_->bound.se.v) >> 1);

	if (flag_ew) { // �����̸�,
		child_->bound.nw.u = center.u;
		child_->bound.se.u = parent_->bound.se.u;
	}
	else { // ����
		child_->bound.nw.u = parent_->bound.nw.u;
		child_->bound.se.u = center.u;
	}

	if (flag_sn) { // �����̸�,
		child_->bound.nw.v = center.v;
		child_->bound.se.v = parent_->bound.se.v;
	}
	else { // �����̸�,
		child_->bound.nw.v = parent_->bound.nw.v;
		child_->bound.se.v = center.v;
	}
};

// ==================== ���� ====================
// - �⺻������, ������ elem_q_->next = NULL�̾�� �Ѵ�.
// - nd_�� reference �� ���� �� ����� ã�ƺ���.
// - child�� �ϳ��� ���ܾ� �ϴ� ��Ȳ�̸�, �Ѳ����� 4���� �������� �Ҵ�����.
// non-max depth -> bridge or leaf. (bridge�� �� �Ȱ���, leaf�� nonmax leaf)
// max depth     -> leaf.
//
//                     [���� �� ]
//                     [�Ǵ�Ʈ��]
//                       /    \
//                      /      \
//                     /        \
//         [�ִ� ���� O]         [�ִ� ���� X]
//          ������ leaf          leaf or bridge
//          /  \                            /   \
//         /    \                          /     \
//  [active O]   [active X]         [leaf]        [bridge]
//   append    activate+append    bridge�� ����,   �׳� �ڽ����� ����.
//                                ���� �ִ� ����
//                                �Բ� �ڽ�����.
//
//      | 00 | 01 |   | HL | HR |
//      |----|----| = |----|----| : LSB(�¿�), MSB(����)
//      | 10 | 11 |   | BL | BR |
//
// [Leaf Ư��]
// - ���� ���� O (id_elem  > -1)
// [bridge Ư��]
// - ���� ���� X (id_elem == -1)

// [��尡 ���� �� ��]
// - �� ��尡 ���� �ɶ��� �׻� leaf�̱� ������ �����Ǵ� ������ �������.
// - leaf�� ���� �� ����, 
//   -> isleaf = 1, id_elem = #, center�� half�� ���ݶ����� �־������.
//
// [leaf -> bridge]
//   -> node�� ����ִ� elem + ���� �� elem�� ���� node�� �ڽĳ��� travel �ؾ��Ѵ�.
//   -> isleaf = 0, id_elem = -1.

 // �̰͵� ��� stack������� ���� �� ���� �� ������ ...
void QuadTreeFastPooled::insert(const int& depth_, Elem* elem_q_, Node* nd_) {
	// 1) �ִ� ���̰� �ƴ� ���.
	if (depth_ < MAX_DEPTH) {
		// leaf Ȥ�� bridge�̴�.
		if (nd_->isleaf) {
			// leaf�� ��� -> bridge�� �ٲ��, 
			// ������+������ �� �� ���� ���ϴ� ���� �ڽ������� ����.
			// 1. isleaf = false. ������ bridge�ɰ���.
			nd_->isleaf = false; // bridge ���� �Ϸ�.

								 //2 . ���� node�� �ִ� elem�� �����͸� �̾ƿ���, node�� header�� -1�� �ٲ�.
			Elem* elem_temp = this->elem_vector[nd_->header_elem];
			nd_->header_elem = -1;

			// TODO: memorypool�� �ٲ���.
			// 3. children�� ������ �Ҵ� (�� �Ϻ��ʱ�ȭ)�� �����Ѵ�. 
			// parent, first_child�޸� �ʱ�ȭ, isleaf=0,  header_elem = -1;
			//nd_->first_child = (Node*)malloc(sizeof(Node) * 4);
			nd_->first_child = objpool_node->getObjectQuadruple();
			node_vector.push_back(nd_->first_child);
			node_vector.push_back(nd_->first_child + 1);
			node_vector.push_back(nd_->first_child + 2);
			node_vector.push_back(nd_->first_child + 3);

			nd_->initializeChildren();

			// 4. elem_temp�� travel.
			int flag_ew, flag_sn, num_child; // �����̸� flag_ew: 1, �����̸� flag_sn: 1
			selectChild(nd_, elem_temp, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			Node* child = (nd_->first_child + num_child); // ���õ� �ڽ� ����.

														  // ���� bridge�����Ƿ�, �ڽ� üũ �ʿ� X. �� leaf�� ������ش�.
			child->isleaf = true;
			child->depth = depth_ + 1;
			// �ڽ� ��尡 �� �༮�� �𼭸� ���� ������ش�.
			calcQuadrantRect(nd_, child, flag_ew, flag_sn);
			// header_elem�� ���� ������ id�� �ְ�, ���� ����� element 1���� �ø���.
			child->header_elem = elem_temp->id;


			// 5. elem_q_�� travel. ���⼭�� ���ϴ°��� �ڽ��� �ִ��� ������ Ȯ���ؾ��Ѵ�.
			selectChild(nd_, elem_q_, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			child = (nd_->first_child + num_child);
			// �ڽ��� leaf����, �ƴϸ� �ƹ� �Ҵ��� �Ǿ����� ������ �׽�Ʈ�ؾ���.
			if (!child->isleaf) {
				child->isleaf = true; // leaf�� �������.
				child->depth = depth_ + 1;
				// �ڽ� ��尡 �� �༮�� �𼭸� ���� ������ش�.
				calcQuadrantRect(nd_, child, flag_ew, flag_sn);
				child->header_elem = elem_q_->id;
			}

			// child node�� ä���� �������� �׳� �Ʒ��� ����
			else insert(depth_ + 1, elem_q_, child);
		}
		else {
			// bridge�� ��� -> �ٷ� �ڽ����� ���� �ȴ�. 
			// ���ϴ� ���� �ڽ��� �ִ� ��� / ���� ���� ����.
			int flag_ew, flag_sn, num_child; // �����̸� flag_ew: 1, �����̸� flag_sn: 1
			selectChild(nd_, elem_q_, flag_ew, flag_sn);
			num_child = (flag_sn << 1) + flag_ew;
			Node* child = nd_->first_child + num_child;

			// �ڽ���( bridge �Ǵ� leaf����), �ƴϸ� �ƹ� �Ҵ��� �Ǿ����� ������ �׽�Ʈ�ؾ���.
			// �ش� �ڽĳ�尡 �ʱ�ȭ�Ǿ����� �ʾҴٸ�, depth = 0�̴�. depth =0�� root���� ����.
			if (!child->depth) {
				child->isleaf = true; // leaf�� �������.
				child->depth = depth_ + 1;
				// �ڽ� ��尡 �� �༮�� �𼭸� ���� ������ش�.
				calcQuadrantRect(nd_, child, flag_ew, flag_sn);
				child->header_elem = elem_q_->id;
			}

			// child node�� ä���� �������� �׳� �Ʒ��� ����
			else insert(depth_ + 1, elem_q_, child);
		}
	}
	// 2) �ִ� ������ ��쿡�� ������ 1�� �̻��� ���Ҹ� ���� leaf�̱� ������, �׳� ���̸�ȴ�.
	else {
		// ���� leaf�� header�̴�.
		// ���� ���� ���ٸ�,���� leaf�� elem�� ������ ���� ã�´�.
		Elem* elem_ptr = this->elem_vector[nd_->header_elem]; 
		while (elem_ptr->next) elem_ptr = elem_ptr->next; 
		elem_ptr->next = elem_q_; // next�� elem_q_�� ����Ǿ���!
	}
};

void QuadTreeFastPooled::insertPublic(Point2<double>& pt_, int& id_, int& id_pts_) {
	Elem* elem_temp = objpool_elem->getObject();
	elem_temp->pt = pt_;
	elem_temp->id = id_; // ���������� �ű�� elem�� id��.
	elem_temp->id_pts = id_pts_; // �ܺ������� �Ű��� id_pts.
	elem_temp->next = nullptr;
	elem_vector.push_back(elem_temp);

	// insert this element.
	insert(0, elem_temp, root);
};

void QuadTreeFastPooled::findNearestElem(Node*& nd_, Elem*& elem_q_, int& id_matched, Node*& nd_matched_) {
	Elem* elem_ptr = this->elem_vector[nd_->header_elem]; // ���� leaf�� header�̴�.
														  // ���� ������ �Ÿ��� ���ϰ�, ���� ������ �Ѱ��ش�. ���� ���� nullptr�̸� ����.
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
	Elem* elem_ptr = this->elem_vector[nd_->header_elem]; // ���� leaf�� header�̴�.
														  // ���� ������ �Ÿ��� ���ϰ�, ���� ������ �Ѱ��ش�. ���� ���� nullptr�̸� ����.
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

// node ���ο� query ���� ���� min_dist�� �̷���� ���� ��������?
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

// node ���ο� ���� �����ϴ��� Ȯ��.
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
	// ��
	double min_dist_scale = *this->min_dist*scale2;
	if (elem_q_->pt.u < nd_->bound.nw.u)
		// �»�
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u)
			+ (elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// ����
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u);
	// ����
		else
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.nw.u)*(elem_q_->pt.u - nd_->bound.nw.u)
			+ (elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
	// ��
	else if (elem_q_->pt.u < nd_->bound.se.u)
		// �߻�
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// ������ ����.
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return true; // ������ ��ģ��.
						 // ����
		else
			return min_dist_scale >
			(elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
	// ��
	else
		// ���
		if (elem_q_->pt.v < nd_->bound.nw.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u)
			+ (elem_q_->pt.v - nd_->bound.nw.v)*(elem_q_->pt.v - nd_->bound.nw.v);
	// ����
		else if (elem_q_->pt.v < nd_->bound.se.v)
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u);
	// ����
		else
			return min_dist_scale >
			(elem_q_->pt.u - nd_->bound.se.u)*(elem_q_->pt.u - nd_->bound.se.u)
			+ (elem_q_->pt.v - nd_->bound.se.v)*(elem_q_->pt.v - nd_->bound.se.v);
};
bool QuadTreeFastPooled::BOBTest(Node*& nd_, Point2<double>& pt_q_) {
	// ��
	double min_dist_scale = *this->min_dist*scale2;
	if (pt_q_.u < nd_->bound.nw.u)
		// �»�
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// ����
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u);
	// ����
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.nw.u)*(pt_q_.u - nd_->bound.nw.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// ��
	else if (pt_q_.u < nd_->bound.se.u)
		// �߻�
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// ������ ����.
		else if (pt_q_.v < nd_->bound.se.v)
			return true;
	// ����
		else
			return min_dist_scale >
			(pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
	// ��
	else
		// ���
		if (pt_q_.v < nd_->bound.nw.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.nw.v)*(pt_q_.v - nd_->bound.nw.v);
	// ����
		else if (pt_q_.v < nd_->bound.se.v)
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u);
	// ����
		else
			return min_dist_scale >
			(pt_q_.u - nd_->bound.se.u)*(pt_q_.u - nd_->bound.se.u)
			+ (pt_q_.v - nd_->bound.se.v)*(pt_q_.v - nd_->bound.se.v);
};


int QuadTreeFastPooled::searchNNSingleQuery(Point2<double>& pt_q_, Node*& node_matched_) {
	// root�� stack ó���� �ִ´�.
	stack.push(this->root);

	// �Ÿ� ���� �ʱ�ȭ
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
		// leaf node�̸� ���� ����� ���� ã�´�.
		if (node->isleaf) {
			stack.total_access++;
			findNearestElem(node, pt_q_, id_elem_matched, node_matched_);
			if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) break;
		}
		// leaf node�� �ƴϸ�, ��� ���� �� �� �����Ѵ�.
		else {
			// ������ node�� BOB �������� ������, �ڽ� ���� �� �ʿ䰡 ����.
			if (BOBTest(node, pt_q_)) {
				stack.total_access++;
				selectChild(node, pt_q_, flag_ew, flag_sn);

				child = node->first_child + (!flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (!flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				// ���� child.
				child = node->first_child + (flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
			}
		}
	}

	stack.clear(); // stack �ʱ�ȭ. (�޸� ���븸 �ʱ�ȭ)
	return id_elem_matched;
};

int QuadTreeFastPooled::searchNNSingleQueryCached(Point2<double>& pt_q_, Node*& node_cached_and_matched_) {
	// 0) �Ÿ� ���� �ʱ�ȭ
	*this->min_dist = 1e15;
	*this->dist_temp = 0;
	stack.total_access++; // node �湮������, counter �ø�.
	int id_elem_matched = -1; // ��Ī�� ���� index
	Node* node = node_cached_and_matched_; // �켱 cached node�� �ҷ��´�.

	// root�� �������� �� single.
	if(node == this->root){
		id_elem_matched = searchNNSingleQuery(pt_q_, node_cached_and_matched_);
		goto flagFinish;
	}
	node_cached_and_matched_ = nullptr;  // ��Ī�� ���� �����ϴ� leaf node�� ������.
										 // 1) nd_cached_���� ������ q�� ���� ����� ���� ã�´�. (������ leaf���� �Ѵ�...)
										 // ����, q�� cached node�� bound �ȿ� ��ġ�ϰ�, BWB���� �����ϸ�, �׳� ��!
										 // ����, inbound �ε�, BWB�� �������� �ʾҴ� ���̶��, parent�� ���ٰ� �ٽ� ����� �����õ�...
	findNearestElem(node, pt_q_, id_elem_matched, node_cached_and_matched_);

	if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) {
		goto flagFinish;
	}

	// 2) BWBTest == true �� bridge���� �ö󰣴�.
	node = node->parent;
	while (true) {
		//if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) break;
		if (inBound(node, pt_q_)) break; // PWB Test only.

		if (node->parent == nullptr) break; // root
		node = node->parent;
		stack.total_access++;
	}

	// 3) ���� ��ġ�� node���� �Ʒ��� ������ search ����.
	stack.push(node);
	// �Ÿ� ���� �ʱ�ȭ
	int flag_ew, flag_sn;
	Node* child = nullptr;
	while (stack.size > 0) {
		node = stack.top();
		stack.pop();
		// leaf node�̸� ���� ����� ���� ã�´�.
		if (node->isleaf) {
			stack.total_access++;
			findNearestElem(node, pt_q_, id_elem_matched, node_cached_and_matched_);
			// if (inBound(node, pt_q_) && BWBTest(node, pt_q_)) goto Finish;
			if (inBound(node, pt_q_)) goto flagFinish; // PWB Test only.

		}
		// leaf node�� �ƴϸ�, ��� ���� �� �� �����Ѵ�.
		else {
			// ������ node�� BOB �������� ������, �ڽ� ���� �� �ʿ䰡 ����.
			// BOB�� �����ϸ�, �ڽ� ���� ���Ѵ�.
			if (BOBTest(node, pt_q_)) {
				stack.total_access++;
				selectChild(node, pt_q_, flag_ew, flag_sn);
				// ���� ������ child�� �ƴ� children�� ���߿� ����Ǿ�� �ϴ�, ���� stack�� ��.
				// ���� != nullptr && BOB pass �� ���鸸 ����.
				// �ٸ� child��, �ʱ�ȭ �� ���� depth�� ������ >0 �̴�.
				// �ʱ�ȭ ���� ���� ���� 0���� �Ǿ��ִ�. root �����ϰ�� 0�� ����������.
				child = node->first_child + (!flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				child = node->first_child + (!flag_sn << 1) + !flag_ew;
				if (child->depth) stack.push(child);
				// ���� child.
				child = node->first_child + (flag_sn << 1) + flag_ew;
				if (child->depth) stack.push(child);
			}
		}
	}

	// ������ �� ����Ͽ�����, top�� 0���� ������ش�. (������ �����͸� ���� �ʿ�� ����.)
flagFinish:
	stack.clear();
	return id_elem_matched;
};


void QuadTreeFastPooled::searchNNMultipleQueries(std::vector<Point2<double>>& pts, std::vector<int>& index_matched, std::vector<Node*>& nodes_matched) {
	int n_pts = pts.size();

	for (int i = 0; i < n_pts; i++) {
		if (inBound(this->root, pts[i])) { // �̹��� ���ο� ������ search
			index_matched[i] = searchNNSingleQuery(pts[i], nodes_matched[i]);
		}
		else { // �̹��� ���ΰ� �ƴϸ�, �Ⱒ.
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
		if (inBound(this->root, pts[i])) { // �̹��� ���ο� ������ search
			index_matched[i] = searchNNSingleQueryCached(pts[i], nodes_cached_and_matched[i]); // cached node�� �Է�.
		}
		else { // �̹��� ���ΰ� �ƴϸ�, �Ⱒ.
			index_matched[i] = -2;
			nodes_cached_and_matched[i] = this->root;
		}
	}
};

void QuadTreeFastPooled::showAllNodes() {
	// ù��°�� ������ root node�̴�.
	Node* node_temp = this->root;
	node_temp->showNodeSpec();
	for (int i = 1; i < node_vector.size(); i++) {
		node_temp = node_vector[i];
		if(node_temp->parent != nullptr)
			node_temp->showNodeSpec();
	}
};

#endif