#ifndef _COMMONSTRUCT_H_
#define _COMMONSTRUCT_H_

#include <iostream>
#include <vector>
#include <memory>

#define PI 3.141592653589793238
#define D2R 0.017453292519943
#define R2D 57.295779513082320

// ����ϱ� ���ϰ� ��������.
typedef unsigned short ushort; // 2����Ʈ
typedef unsigned char uchar; // 1����Ʈ

// double �� ushort�� �� �� ���� �������̽��� ����ϱ� ���� ���ø����� ����.
template <typename T> struct Point2 { // 2���� �� (feature)
	T u;
	T v;
	Point2() : u(0), v(0) {}; // �⺻ ������
	Point2(T u_, T v_) : u(u_), v(v_) {};
};

template <typename T> struct Point3 { // 3���� �� (3d point)
	T x;
	T y;
	T z;
	Point3() : x(0), y(0), z(0) {}; // �⺻ ������
	Point3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {};
};

struct Bound {
	Point2<ushort> nw; // �»��.
	Point2<ushort> se; // ���ϴ�.
	Bound() : nw(Point2<ushort>(0, 0)), se(Point2<ushort>(0, 0)) {}; // �⺻ ������
	Bound(Point2<ushort> nw_, Point2<ushort> se_) :nw(nw_), se(se_) {};
};

// �̹����� ũ�Ⱑ 65536 �̻��� ���, �������� ����.
// �ٷ� ������ ���� 4���� children ��, ù��° child�� ����Ű�� �ּ� ��.
// 1,2,3,4 ������� HL, HR, BL, BR. (�»�, ���, ����, ����)
// Z �������
//
// | 00 | 01 |   | HL | HR |
// |----|----| = |----|----| : LSB(�¿�), MSB(����)
// | 10 | 11 |   | BL | BR |
//
// first_child + 0: HL, first_child + 1: HR, first_child + 2: BL, first_child + 3: BR
// 64 bits �ü���̹Ƿ� 8 bytes�� �ּҸ� ���� �� �� �ִ�.
struct Node { // size = 32 bytes
	Node* parent;    // 8 bytes
	Node* first_child; // 8 first_child + 1: HR, first_child + 2: BL, first_child + 3: BR
	Bound bound;     // 8 bytes  0 ~ 65536 ������ �̹��� ��ǥ�� �Ҽ������� �������� ū �ǹ̾���.
	uchar depth;     // 1, ���̵� ����ܰ�� �������� �ʱ� ������ 0~255�̸� ���.
	bool isleaf;     // 1, leaf���� �ƴ��� �˷���.
	bool isvisited;     // 1, �湮�ߴ��� �ƴ��� �˷���. (cached version�� ����)
	int header_elem; // 4 ���� leaf����̸�, ����Ʈ ����Ʈ�� ����� ����Ʈ�� �ּҷ� �����Ѵ�.

	// ���ο� ��� ������.
	Node() : parent(nullptr), depth(0), isleaf(false), isvisited(false),
		bound(Bound(Point2<ushort>(-1, -1), Point2<ushort>(-1, -1))), header_elem(-1) {};

	Node(Node* parent_, int depth_, bool isleaf_)
		: parent(parent_), depth(depth_), isleaf(isleaf_), isvisited(false),
		bound(Bound(Point2<ushort>(-1, -1), Point2<ushort>(-1, -1))), header_elem(-1) {};

	// � ��忡 �ڽ��� �����µ�, query�� �����ϸ� �����ϴ� �Լ�.
	void initializeChildren() {
		// �޸� �Ҵ�. ����ִ� �ڽĵ��� �ʱ�ȭ���ش�.
		// this->first_child = (Node*)malloc(sizeof(Node) * 4);
		// ��ü �迭 �����Ҵ��� �ʱ�ȭ�� �Ұ����ϴ�.. �⺻�����ڿ��� �ް��Ѱ� �ذ��ؾ��Ѵ�.
		for (int i = 0; i < 4; i++) {
			(this->first_child + i)->parent = this;
			(this->first_child + i)->first_child = nullptr;
			(this->first_child + i)->depth = 0;
			(this->first_child + i)->isvisited = 0;
			(this->first_child + i)->isleaf = 0;
			(this->first_child + i)->header_elem = -1;
		}
	};

	void showNodeSpec() {
		std::cout << "node spec.\n";
		std::cout << "lvl: " << (int)this->depth;
		std::cout << ", parent: " << this->parent;
		std::cout << ", this  : " << this;
		std::cout << ", bound: [" << this->bound.nw.u << "," << this->bound.nw.v <<
			"] / [" << this->bound.se.u << "," << this->bound.se.v << "]";
		std::cout << ", leaf: " << this->isleaf;
		std::cout << ", visit: " << this->isvisited;
		std::cout << ", 1stelem: " << this->header_elem << std::endl;
	}
};

// node���� �����ϴ� stack���δٰ� ������.
template <typename T>
class PointerStack {
public:
	int size;
	int MAX_SIZE;
	int total_access;
	T** values; // ���� �Ҵ� ��, ������ �迭�̶�� ���� �ȴ�.
	PointerStack() {
		MAX_SIZE = 65536; // stack�� �׷��� ũ�� �ʾƵ� �Ǵ���.
		values = (T**)malloc(sizeof(T*)*MAX_SIZE); // 8 * max_size
		size = 0; // ���� �ƹ��͵� �� ����.
		total_access = 0; // �� ����̳� ��忡 �����ߴ���? leaf�� BOB ����� ��忡 ���ؼ��� ���. 
						  // �� ��Ʈ�� ������ ��Ī�ϱ��������� �ʱ�ȭ X.
	}
	~PointerStack() { delete[] values; }// �����ߴ� stack ����.
	void push(T* value_) {
		if (!isFull()){
			*(values + size) = value_;
			++size;
		}
		else printf("[ERROR]: NodeStack is full.\n");
	}
	void pop() {
		if (!empty()) --size;
		else printf("[ERROR]: NodeStack is empty.\n");
	}
	T* top() {
		if (!empty()) return *(values + (size - 1));
		else return nullptr;
	}
	// ���� �ִ��� ������ Ȯ��.
	bool empty() {
		if (size < 1) return true;
		else return false;
	}
	// �� á���� �ƴ��� Ȯ��.
	bool isFull() { return (size == MAX_SIZE); }
	// stack�� ����ϰ� ������ ���� �ʿ�� ������, size=0���� ���������Ѵ�.
	void clear() { size = 0;}
};

// �� Element�� doubly one-way directed list.
//  ---------     |    2   [|--->
// |        [|--->|---------|
// |    1    | 
// |        [|--->|---------|
//  ---------     |    3   [|--->
//
// (1) 1���� ���⸸ �����ϴ� ����. 
// first_next > 0 & second_next == nullptr;
// first_dir > -1 & second_dir == -1
// (2) 2���� ������ �����ϴ� ����. 
// second_next > 0 & first_next > 0
// second_dir > first_dir > -1
// -> dir�� ���� ũ�� second�� �̵���Ų��.
struct ElemOverlap { // 40 bytes
					 // ���� ����� Elem 2���� �����Ϳ� ���� �� �� �ִ� ����
					 // first_next: ù��° gradient ����, first_next + 1: �ι�° gradient ����.
	ElemOverlap* first_next;  // 8 bytes, ù��° ����
	ElemOverlap* second_next; // 8 bytes, �ι�° ����, �̰� NULL�� �ƴϸ�, Shared region�� �ִ°��̴�.
	Point2<double> pt;
			         
	int id;       // 4 bytes,  �����ͺ��̽��� ���õ� �� ��ȣ.
					 // first dir�� �׻� second_dir���� ���������� �Ѵ�.
	char first_dir;  // 1 byte
	char second_dir; // 1 byte

	ElemOverlap() : first_next(nullptr), second_next(nullptr), pt(Point2<double>(-1, -1)), first_dir(-1), second_dir(-1), id(-1) {}; // �⺻ ������, �Ⱦ���

	ElemOverlap(double& u_, double& v_, char& dir_, int& id_) 
		: first_next(nullptr), second_next(nullptr), pt(Point2<double>(u_, v_)), first_dir(dir_), second_dir(-1), id(id_) {}; // �ʱ�ȭ ������ (dir 1��)
	ElemOverlap(Point2<double>& pt_, char& dir_, int& id_)
		: first_next(nullptr), second_next(nullptr), pt(pt_), first_dir(dir_), second_dir(-1), id(id_) {}; // �ʱ�ȭ ������ (dir 1��)

	ElemOverlap(double& u_, double& v_, char& dir1_, char& dir2_, int& id_)  // �ʱ�ȭ ������ (dir 2��)
		: first_next(nullptr), second_next(nullptr), pt(Point2<double>(u_, v_)), id(id_) 
	{
		if (dir1_ < dir2_) {
			first_dir = dir1_; 
			second_dir = dir2_;
		}
		else {
			first_dir = dir2_;
			second_dir = dir1_;
		}
	};
	ElemOverlap(Point2<double>& pt_, char& dir1_, char& dir2_, int& id_)  // �ʱ�ȭ ������ (dir 2��)
		: first_next(nullptr), second_next(nullptr), pt(pt_), id(id_)
	{
		if (dir1_ < dir2_) {
			first_dir = dir1_;
			second_dir = dir2_;
		}
		else {
			first_dir = dir2_;
			second_dir = dir1_;
		}
	};
};

struct Elem {   // 32 bytes, singly index linked list. (���� ������ �Ҵ��ؾ��ϴµ�)
	Elem* next; //  8 bytes, ������ ����� ����. NULL�̸� ����� ���Ұ� ����.
	Point2<double> pt;   // 16 bytes, ���� ���� ��ǥ.
	int id;     // 4 bytes, // �ش� element�� id ��ȣ.
	int id_pts; // 4 bytes, // std::vector<Point2d<double>> points ������ id ��ȣ. not necessarily id == id_pts

	Elem() : next(nullptr), pt(Point2<double>(-1, -1)), id(-1) {}; // �⺻������
	Elem(double& u_, double& v_) : next(nullptr), pt(Point2<double>(u_, v_)), id(-1) {};
	Elem(Point2<double>& pt_) : next(nullptr), pt(pt_), id(-1) {};
};

#endif