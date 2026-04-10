#ifndef __TBB_CONCURRENT_QUEUE__
#define __TBB_CONCURRENT_QUEUE__

#include "tbb/concurrent_queue.h"
#include "tbb/spin_mutex.h"

template <class T>
class TBBConcurrentQueue
{
public:
	TBBConcurrentQueue(void)
	{}

	~TBBConcurrentQueue(void)
	{}

	void SetState(Thread_queue_states stat)
	{
		tbb::spin_mutex::scoped_lock lock(Mutex);
		state = stat;
	}

	Thread_queue_states GetState()
	{
		return state;
	}

	// Add an element to the queue.
	void Enqueue(T t)
	{
		queue.push(t);
	}

	bool empty()
	{
		return queue.empty();
	}

	bool Dequeue(T& val)
	{
		if (queue.empty())
			return false;
		return queue.try_pop(val);
	}

private:
	tbb::concurrent_queue<T> queue;
	//tbb::concurrent_bounded_queue<T> queue;
	tbb::spin_mutex Mutex;
	Thread_queue_states state;
};

#endif