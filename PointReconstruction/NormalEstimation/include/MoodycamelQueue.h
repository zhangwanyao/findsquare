#ifndef __MOODYCAMEL_QUEUE__
#define __MOODYCAMEL_QUEUE__

#include "concurrentqueue.h"
#include "tbb/spin_mutex.h"

template <class T>
class MoodyCamelQueue
{
public:
	MoodyCamelQueue(void)
	{
		//queue.initialBlockPoolSize = 4096;
	}

	~MoodyCamelQueue(void)
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
		queue.enqueue(t);
	}

	bool Dequeue(T& val)
	{
		return queue.try_dequeue(val);
	}

private:
	moodycamel::ConcurrentQueue<T> queue;
	tbb::spin_mutex Mutex;
	Thread_queue_states state;
};

#endif