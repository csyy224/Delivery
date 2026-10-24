#ifndef MINHEAP_H
#define MINHEAP_H

#include <vector>
#include <stdexcept>

struct HeapNode
{
	int id;
	double dist;
	bool operator<(const HeapNode& other) const
	{
		return dist < other.dist;
	}
};

class MinHeap
{
	private:
	std::vector<HeapNode> heap;

	void siftUp(int index)
	{
		while (index > 0)
		{
			int parent = (index - 1) / 2;
			if (heap[index] < heap[parent])
			{
				std::swap(heap[index], heap[parent]);
				index = parent;
			}
			else
			{
				break;
			}
		}
	}

	void siftDown(int index)
	{
		int size = heap.size();
		while (index < size)
		{
			int leftChild = 2 * index + 1;
			int rightChild = 2 * index + 2;
			int smallest = index;

			if (leftChild < size && heap[leftChild] < heap[smallest])
			{
				smallest = leftChild;
			}
			if (rightChild < size && heap[rightChild] < heap[smallest])
			{
				smallest = rightChild;
			}
			if (smallest != index)
			{
				std::swap(heap[index], heap[smallest]);
				index = smallest;
			}
			else
			{
				break;
			}
		}
	}

	public:
	MinHeap() = default;

	void push(const HeapNode& node)
	{
		heap.push_back(node);
		siftUp(heap.size() - 1);
	}

	void pop()
	{
		if (heap.empty())
		{
			throw std::out_of_range("Heap is empty");
		}
		std::swap(heap.front(), heap.back());
		heap.pop_back();
		siftDown(0);
	}

	HeapNode top() const
	{
		if (heap.empty())
		{
			throw std::out_of_range("Heap is empty");
		}
		return heap.front();
	}

	bool empty() const { return heap.empty();}
	size_t size() const { return heap.size(); }
};
#endif
