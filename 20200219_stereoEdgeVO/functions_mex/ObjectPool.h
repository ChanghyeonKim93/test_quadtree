#ifndef _OBJECTPOOL_H_
#define _OBJECTPOOL_H_

#include <iostream>
#include <queue>
#include <memory>

// Attaboy!

// �ش� ObjectPool�� ���ӵ� ������ �Ҵ��ϰ�, stack ������� �޸𸮸� �Ҵ��ϴ� �����̴�.
// ������ ������Ÿ�� T�� pool�� �����ϴ� �Լ�.
// new T() �⺻ �����ڸ� ����������Ѵ�.

// �Ŵ��� memory chunk. static���� �����ϸ� ObjectPool�̶�� class�� ����ϴ� ��� 
// class�� ���� memory_chunk��� ������ ��ӹ޾� �����Ѵ� ...
// 20200215 - ���� ������� �ʾƵ� �Ǵ°�? ������ static member�� �׳� �ٷ� ����ϱ�;
// �� ��ü�� �������� ����.
// �� N���� object�� �����Ѵٰ� �ϸ�, N-1, N-2, ... , 2, 1, 0 ������ ���ÿ� ����.
template <typename T>
class ObjectPool {
private:
	T* _memory_chunk; // ���� class�� template���� ������, ������ _memory_chunk�� �����Ѵ�. 
	std::queue<T*> _obj_queue; // �Ҵ��ؾ� �� �޸��� �����͸� ������ �ִ�.

public:
	// ������
	ObjectPool(size_t max_num_object) {
		_memory_chunk = (T*)malloc(sizeof(T)*max_num_object);
		for (int i = 0; i < max_num_object; i++) _obj_queue.push(_memory_chunk + i);
		printf("object size: %d / allocated # of objects: %d / total memory consumption: %d [Mbytes]\n",
		sizeof(T), _obj_queue.size(), (sizeof(T)*_obj_queue.size()) / (1024 *1024));
	};

	// memory chunk�� �պκ��� �����´�.
	T* getObject() {
		if (!_obj_queue.empty()) {
			T* ptr = _obj_queue.front();
			_obj_queue.pop();
			return ptr;
		}
		else {
			printf("ERROR: no object pool is remaining.\n");
			return nullptr;
		}
	}

	T* getObjectQuadruple() {
		if (!_obj_queue.empty()) {
			T* ptr = _obj_queue.front();
			_obj_queue.pop();
			_obj_queue.pop();
			_obj_queue.pop();
			_obj_queue.pop();
			return ptr;
		}
		else {
			printf("ERROR: no object pool is remaining.\n");
			return nullptr;
		}
	}
	
	void returnObject(T* ptr_) {
		_obj_queue.push(ptr_);
	}

	// �Ҹ���
	~ObjectPool() {
		if(_memory_chunk != nullptr){
			free(_memory_chunk);
			printf("\n Memory chunk in object pool is successfully returned.\n");
		}
		else{
			printf("\n Already empty.\n");
		}
	};

	void showRemainedMemory(){
		printf("remained mem: %d\n", _obj_queue.size());
	};

	// ������ new, delete�� overloading �ҷ��ߴµ� ... ���ߴ� ��
	//	T* getMemory();
	//void retunrMemory(T* ptr);
	/*void* operator new(size_t size)
	{
		return GObjectPool::getInstance()->AllocObject(size);
	}
	void operator delete(void* pointer, size_t size)
	{
		GObjectPool::getInstance()->FreeObject(pointer, size);
	}*/
};

// // ������������� �ʱ�ȭ.
// template <typename T>
// T* ObjectPool<T>::_memory_chunk = nullptr;
// template <typename T>
// size_t ObjectPool<T>::_used_count = 0;
// template <typename T>
// std::queue<T*> ObjectPool<T>::_obj_queue;

#endif