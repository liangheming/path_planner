#include <queue>
#include <iostream>
using Element = std::pair<float, unsigned int>;

struct PairCompare
{
    bool operator()(const Element &a, const Element &b) const
    {
        return a.first > b.first;
    }
};

using MinHeap = std::priority_queue<Element, std::vector<Element>, PairCompare>;

int main()
{
    MinHeap heap;
    heap.push(std::make_pair(2.0, 1));
    heap.push(std::make_pair(1.0, 2));
    heap.push(std::make_pair(1.5, 2));
    heap.push(std::make_pair(1.3, 2));
    while (!heap.empty())
    {
        std::cout << heap.top().first << std::endl;
        heap.pop();
    }
}