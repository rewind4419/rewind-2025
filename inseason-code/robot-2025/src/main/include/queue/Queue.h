#pragma once

#define MAX_TASKS 1024
#define MAX_INSTANT_RUN 8

class Task
{
public:
	virtual ~Task() {};
	virtual void Start() {};
	virtual bool Loop() { return true; };
	virtual void End() {};
};

// Once you add a task, the queue takes ownership of the pointer. 
// Once the task finishes executing, it is freed.
// Also, all tasks are freed if the queue gets deallocated
class Queue
{
public:
	Task* tasks[MAX_TASKS];

	unsigned int currentTask = 0;
	unsigned int endTask = 0;

	bool hasCurrentTaskStarted = false;


	// Allocate the task on the heap, then pass it here. It will be destructed & deallocated by the task queue after its done executing.
	~Queue();
	void AddTask(Task* task);
	void Update();
	bool IsBusy();
	int TaskCount();
	void Clear();
};
