#ifndef CIRCULARQUEUE_H
#define CIRCULARQUEUE_H

#include "Parcel.h"
#include <vector>
#include <stdexcept>

class CircularQueue
{
	private:
	std::vector<Parcel> queue;
	size_t front, rear, count;

	void resize(size_t new_capacity)
	{
		std::vector<Parcel> new_queue(new_capacity);
		for (size_t i = 0; i < count; ++i)
			new_queue[i] = queue[(front + i) % queue.size()];
		queue = std::move(new_queue);
		front = 0;
		rear = count;
	}

	public:
	CircularQueue(size_t capacity = 10) : queue(capacity), front(0), rear(0), count(0) {}
	bool empty() const { return count == 0; }
	bool full() const { return count == queue.size(); }
	size_t size() const { return count; }

	void enqueue(const Parcel& p)
	{
		if (full())	resize(queue.size() * 2);
		queue[rear] = p;
		rear = (rear + 1) % queue.size();
		count++;
	}

	void dequeue()
	{
		if (empty()) throw std::runtime_error("Queue is empty");
		front = (front + 1) % queue.size();
		count--;
	}

	Parcel front() const
	{
		if (empty()) throw std::runtime_error("Queue is empty");
		return queue[front];
	}
};
#endif
