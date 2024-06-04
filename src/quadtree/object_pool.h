#ifndef _OBJECT_POOL_H_
#define _OBJECT_POOL_H_
#include <iostream>
#include <memory>
#include <queue>

template <typename T>
class ObjectPool {
 public:
  ObjectPool(int max_num_object) : max_num_object_(max_num_object) {
    size_of_object_ = sizeof(T);
    memory_chunk_ = (T*)malloc(size_of_object_ * max_num_object);
    for (int i = 0; i < max_num_object_; ++i)
      object_queue_.push(memory_chunk_ + i);

    printf(
        "object size: %ld / allocated # of objects: %ld / total memory "
        "consumption: %ld [Mbytes]\n",
        size_of_object_, object_queue_.size(),
        (size_of_object_ * object_queue_.size()) / (1024 * 1024));
  }

  T* GetObject() {
    if (!object_queue_.empty()) {
      T* ptr = object_queue_.front();
      object_queue_.pop();
      return ptr;
    } else {
      throw std::runtime_error("No object pool is remaining!\n");
      return nullptr;
    }
  }

  T* GetObjectQuadruple() {
    if (!object_queue_.empty()) {
      T* ptr = object_queue_.front();
      object_queue_.pop();
      object_queue_.pop();
      object_queue_.pop();
      object_queue_.pop();
      return ptr;
    } else {
      throw std::runtime_error("No object pool is remaining!\n");
      return nullptr;
    }
  }

  void ReturnObject(T* ptr) { object_queue_.push(ptr); }

  ~ObjectPool() {
    if (memory_chunk_ != nullptr) {
      free(memory_chunk_);
      printf(
          "ObjectPool - memory chunk [%f] MBytes in object pool is "
          "successfully returned.\n",
          (double)(sizeof(T) * max_num_object_) / (1024.0 * 1024.0));
    } else {
      printf("ObjectPool - already empty.\n");
    }
  }

  void ShowRemainingMemory() {
    printf("remained mem: %ld\n", object_queue_.size());
  }

 private:
  T* memory_chunk_;
  std::queue<T*> object_queue_;
  int max_num_object_;
  size_t size_of_object_;
};

#endif