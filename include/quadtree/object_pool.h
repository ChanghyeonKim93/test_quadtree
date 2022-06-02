#ifndef _OBJECT_POOL_H_
#define _OBJECT_POOL_H_
#include <iostream>
#include <queue>
#include <memory>

template <typename T>
class ObjectPool {
private:
	T* memory_chunk_;
	std::queue<T*> obj_queue_; 
    uint32_t max_num_object_;

public:
    ObjectPool(uint32_t max_num_object): max_num_object_(max_num_object){
        memory_chunk_ = (T*)malloc(sizeof(T)*max_num_object);
        for(uint32_t i = 0; i < max_num_object_; ++i) obj_queue_.push(memory_chunk_+i);
        
		printf("object size: %ld / allocated # of objects: %ld / total memory consumption: %ld [Mbytes]\n",
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
    };

    ~ObjectPool() {
        if(memory_chunk_ != nullptr){
            free(memory_chunk_);
			printf("ObjectPool - memory chunk [%f] MBytes in object pool is successfully returned.\n",
                    (double)(sizeof(T)*max_num_object_) / (1024.0 *1024.0));
        }
        else{
			printf("ObjectPool - already empty.\n");
        }
    };

    void showRemainingMemory(){
		printf("remained mem: %ld\n", obj_queue_.size());
    };
};

#endif