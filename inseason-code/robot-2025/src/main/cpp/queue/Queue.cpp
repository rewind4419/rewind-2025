#include "queue/Queue.h"

#include <stdio.h>

Queue::~Queue()
{
	for (int i = currentTask; (i % MAX_TASKS) != int(endTask); i = (i + 1) % MAX_TASKS)
	{
		delete tasks[i];
	}
}

void Queue::AddTask(Task* task)
{
	if ((endTask + 1) % MAX_TASKS == currentTask)
	{
		printf("Task queue is full! Aborting AddTask\n");
		delete task;
	}
	else
	{
		this->tasks[endTask] = task;
		endTask = (endTask + 1) % MAX_TASKS;
	}
}

void Queue::Update()
{
	if (currentTask == endTask)
	{
		 //// Task queue has reached the end, nothing to do
		 //printf("Queue not busy\n");
		return;
	}

	if (this->hasCurrentTaskStarted == false)
	{
		// The queue has things queued up and the current task hasn't started yet, start it now.
		this->tasks[currentTask]->Start();
		this->hasCurrentTaskStarted = true;
	}

	bool result = this->tasks[currentTask]->Loop();

	if (result)
	{
		// Task is finished, end it and shift over
		this->tasks[currentTask]->End();
		delete tasks[currentTask];
		currentTask = (currentTask + 1) % MAX_TASKS;
		this->hasCurrentTaskStarted = false;

		// Step through the next tasks, calling their start and loops once
		// If the loop returns true immediately, go on to the next one
		// Cap the amount of loops at a constant
		// If the loop doesn't return true immediately, finish
		for (int i = 0; i < MAX_INSTANT_RUN; i++)
		{
			if (this->currentTask == this->endTask) { break; }

			this->tasks[currentTask]->Start();
			this->hasCurrentTaskStarted = true;
			bool result2 = this->tasks[currentTask]->Loop();
			if (result2 == true)
			{
				// Continue looping through
				this->tasks[currentTask]->End();
				delete tasks[currentTask];
				this->hasCurrentTaskStarted = false;
				currentTask = (currentTask + 1) % MAX_TASKS;
			} 
			else
			{
				break;
			}
		}
	}
}

bool Queue::IsBusy()
{
	if (currentTask == endTask)
	{
		return false;
	}
	else
	{
		return true;
	}
}
