#include "queue/standardTasks.hpp"
#include <chrono>



DelayTask::DelayTask(int duration_ms){
    duration = duration_ms;
}
void DelayTask::start(){
    auto now = std::chrono::system_clock::now();
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>( now.time_since_epoch()).count();
    
    startTime = currentTime;
}
bool DelayTask::loop(){
    auto now = std::chrono::system_clock::now();
    auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>( now.time_since_epoch()).count();

    return currentTime >= startTime + duration;
}
void DelayTask::dispose(){
    delete this;
}


CustomTask::CustomTask(std::function<bool()> func){
    loopFunc = func;
}
bool CustomTask::loop(){
    return loopFunc();
}
void CustomTask::dispose(){
    delete this;
}



TaskList::TaskList(){
    queue = TaskQueue();
}
void TaskList::addTask(Task* task){
    queue.addTask(task);
}
bool TaskList::loop(){
    queue.update();
    return !queue.active;
}
void TaskList::dispose(){
    queue.destroy();
    delete this;
}

ForkTask::ForkTask(Task* task0, Task* task1):task0(task0),task1(task1){}
void ForkTask::start(){
    task0->start();
    task1->start();
}
bool ForkTask::loop(){
    if(!task0Done)
    {
        task0Done = task0->loop();
    }
    if(!task1Done)
    {
        task1Done = task1->loop();
    }
    return task0Done && task1Done;
}
void ForkTask::dispose(){
    task0->dispose();
    task1->dispose();
    delete this;
}
