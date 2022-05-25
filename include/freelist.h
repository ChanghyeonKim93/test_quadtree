#ifndef _FREE_LIST_H_
#define _FREE_LIST_H_
#include <iostream>
#include <vector>

template <class T>
class FreeList
{

public:
    /// Creates a new free list.
    FreeList();

    // Insert an element to the free list and returns an index to it.
    int insert(const T& element);

    // Removes the n-th element from the free list.
    void erase(int n);

    // Removes all elements from the free list.
    void clear();

    // Returns the range of valid indices
    int range() const;

    // Returns the n-th element
    T& operator[](int n); 

    // Returns the n-th element
    const T& operator[](int n) const;

private:
    union FreeElement
    {
        T element;
        int next;
    };

    std::vector<FreeElement> data;
    int first_free;
};


template <class T>
FreeList<T>::FreeList(): first_free(-1)
{
}

template <class T>
int FreeList<T>::insert(const T& element)
{
    if(first_free == -1) 
    {
        FreeElement fe;
        fe.element = element;
        data.push_back(fe);
        return static_cast<int>(data.size() - 1);
    }
    else 
    {
        const int index = first_free;
        data[index].element = element;
        first_free = data[index].next;
        return index;
    }
}

template <class T>
void FreeList<T>::erase(int n)
{
    data[n].next = first_free;
    first_free = n;
}

template <class T>
void FreeList<T>::clear()
{
    data.clear();
    first_free = -1;
}

template <class T>
int FreeList<T>::range() const
{
    return std::static_cast<int>(data.size());
}

template <class T>
T& FreeList<T>::operator[](int n)
{
    return data[n].element;
}

template <class T>
const T& FreeList<T>::operator[](int n) const
{
    return data[n].element;
}


#endif