#ifndef __SAFE_QUEUE__
#define __SAFE_QUEUE__

#include <queue>
#include <mutex>
#include <condition_variable>

enum Thread_queue_states
{
	THREAD_QUEUE_START,
	THREAD_QUEUE_CLOSE
};

typedef std::chrono::duration<int, std::milli> milliseconds_type;

// A threadsafe-queue.
template <class T>
class SafeQueue
{
public:
	SafeQueue(void) : q(), m(), c()
	{}

	~SafeQueue(void)
	{}

	void SetState(Thread_queue_states stat)
	{
		std::lock_guard<std::mutex> lock(m);
		state = stat;
	}

	Thread_queue_states GetState()
	{
		return state;
	}

	// Add an element to the queue.
	void Enqueue(T t)
	{
		std::lock_guard<std::mutex> lock(m);
		q.push(t);
	}

	void Enqueue_nb(T t)
	{
		std::lock_guard<std::mutex> lock(m);
		q.push(t);
		c.notify_one();
	}

	bool empty()
	{
		std::lock_guard<std::mutex> lock(m);
		return q.empty();
	}
	// Get the "front"-element.
	// If the queue is empty, wait till a element is available.
	bool Dequeue(T &val, const std::chrono::milliseconds timeout = milliseconds_type(1))
	{
		std::unique_lock<std::mutex> lock(m);
		if (q.empty())
			return false;
		val = q.front();
		q.pop();
		return true;
	}

	bool Dequeue_nb(T& val, const std::chrono::milliseconds timeout = milliseconds_type(1))
	{
		std::unique_lock<std::mutex> lock(m);
		while (q.empty())
		{
			// release lock as long as the wait and check if queue closed.
			if (c.wait_for(lock, timeout) == std::cv_status::timeout && state == THREAD_QUEUE_CLOSE)
			{
				return false;
			}
			//c.wait(lock);
		}

		val = q.front();
		q.pop();
		return true;
	}

private:
	std::queue<T> q;
	mutable std::mutex m;
	std::condition_variable c;
	Thread_queue_states state;
};
#endif