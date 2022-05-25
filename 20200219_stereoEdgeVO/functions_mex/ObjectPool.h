#ifndef _OBJECTPOOL_H_
#define _OBJECTPOOL_H_

#include <iostream>
#include <queue>
#include <memory>

// Attaboy!

// 해당 ObjectPool은 연속된 공간을 할당하고, stack 기반으로 메모리를 할당하는 구조이다.
// 임의의 데이터타입 T의 pool을 생성하는 함수.
// new T() 기본 생성자를 정의해줘야한다.

// 거대한 memory chunk. static으로 정의하면 ObjectPool이라는 class를 상속하는 모든 
// class는 같은 memory_chunk라는 변수를 상속받아 공유한다 ...
// 20200215 - 굳이 상속하지 않아도 되는걸? 어차피 static member는 그냥 바로 생기니까;
// 걍 객체로 가지도록 하자.
// 총 N개의 object를 저장한다고 하면, N-1, N-2, ... , 2, 1, 0 순으로 스택에 들어간다.
template <typename T>
class ObjectPool {
private:
	T* _memory_chunk; // 같은 class를 template으로 받으면, 무조건 _memory_chunk를 공유한다. 
	std::queue<T*> _obj_queue; // 할당해야 할 메모리의 포인터를 가지고 있다.

public:
	// 생성자
	ObjectPool(size_t max_num_object) {
		_memory_chunk = (T*)malloc(sizeof(T)*max_num_object);
		for (int i = 0; i < max_num_object; i++) _obj_queue.push(_memory_chunk + i);
		printf("object size: %d / allocated # of objects: %d / total memory consumption: %d [Mbytes]\n",
		sizeof(T), _obj_queue.size(), (sizeof(T)*_obj_queue.size()) / (1024 *1024));
	};

	// memory chunk의 앞부분을 가져온다.
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

	// 소멸자
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

	// 원래는 new, delete를 overloading 할려했는데 ... 안했다 걍
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

// // 정적멤버변수들 초기화.
// template <typename T>
// T* ObjectPool<T>::_memory_chunk = nullptr;
// template <typename T>
// size_t ObjectPool<T>::_used_count = 0;
// template <typename T>
// std::queue<T*> ObjectPool<T>::_obj_queue;

#endif