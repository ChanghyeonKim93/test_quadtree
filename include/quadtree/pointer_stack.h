#ifndef _POINTER_STACK_H_
#define _POINTER_STACK_H_
#include <iostream>

// node만을 저장하는 stack으로다가 만들자.
template <typename T>
class PointerStack {
public:
	PointerStack() :MAX_SIZE(65536), size_(0), total_access_(0) {
		values = (T**)malloc(sizeof(T*)*MAX_SIZE); // 8 * max_size
	}
	~PointerStack() { delete[] values_; }
	void push(const T*& value) {
		if (!isFull()){
			*(values_ + size_) = value;
			++size_;
		}
		else throw std::runtime_error("[ERROR]: PointerStack is full.");
	}
	void pop() {
		if (!empty()) --size_;
		else printf("[ERROR]: NodeStack is empty.\n");
	}
	T* top() {
		if (!empty()) return *(values_ + (size_ - 1));
		else return nullptr;
	}
    T* topAndPop(){
        if (!empty()) {
            return *(values_ + (--size_));
        }
        else return nullptr;
    };
	inline bool empty() { return (size_ < 1);};
	inline int size() {return size_;};
    inline bool isFull() { return (size_ == MAX_SIZE); }
	inline void clear() { size_ = 0;}

public:
    uint32_t getTotalAccess() const{ return total_access_; };
    void resetTotalAccess() { total_access_ = 0; };
    void addTotalAccess() {++total_access_;};

private:
	uint32_t size_;
	uint32_t MAX_SIZE;
	uint32_t total_access_;
    T** values_;
};


#endif