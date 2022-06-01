#ifndef _OBJECT_POOL_H_
#define _OBJECT_POOL_H_
#include <iostream>
#include <queue>
#include <memory>

template <typename T>
class ObjectPool {
private:
	T* memory_chunk_; // 같은 class를 template으로 받으면, 무조건 _memory_chunk를 공유한다. 
	std::queue<T*> obj_queue_; // 할당해야 할 메모리의 포인터를 가지고 있다.

public:
    ObjectPool(size_t max_num_object){
        memory_chunk_ = (T*)malloc(sizeof(T)*max_num_object);
        for(uint32_t i = 0; i < max_num_object; ++i) obj_queue_.push(memory_chunk_+i);
        
		printf("object size: %d / allocated # of objects: %d / total memory consumption: %d [Mbytes]\n",
		sizeof(T), obj_queue_.size(), (sizeof(T)*obj_queue_.size()) / (1024 *1024));
    };

    T* getObject() {
        if( !obj_queue_.empty()){
            T* ptr = obj_queue_.front();
            obj_queue_.pop();
            return ptr;
        }
        else{
            throw std::runtime_error("No object pool is remaining!\n");
            return nullptr;
        }
    };

    T* getObjectQuadruple() {
        if(!obj_queue_.empty()){
            T* ptr = obj_queue_.front();
            obj_queue_.pop();
            obj_queue_.pop();
            obj_queue_.pop();
            obj_queue_.pop();
            return ptr;
        }
        else{
            throw std::runtime_error("No object pool is remaining!\n");
            return nullptr;
        }
    };

    void returnObject(T* ptr){
        obj_queue_.push(ptr);
    }

    ~ObjectPool() {
        if(memory_chunk_ != nullptr){
            free(memory_chunk_);
			printf("\n Memory chunk in object pool is successfully returned.\n");
        }
        else{
			printf("\n Already empty.\n");
        }
    };

    void showRemainingMemory(){
		printf("remained mem: %d\n", obj_queue_.size());
    };
};

#endif