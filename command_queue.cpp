// command_queue.cpp
// a simple <int> queue to store the commands for the RoverC

// FIFO queue
// queues are a type of container adaptor, specifically designed to 
// operate in a FIFO context (first-in first-out), where elements are 
// inserted into one end of the container and extracted from the other.
// see: http://www.cplusplus.com/reference/queue/queue/
// or: https://www.thegeekstuff.com/2017/01/cpp-stl-queue/
#include <queue> 

// define the queue object
std::queue<int> Command_queue;

// add an int value to the end of the queue
void code_queue_add(int val){
  Command_queue.push(val);
}

// get the first value out of the queue
// return false if there is no element in the queue anymore
bool code_queue_get(int *val){
  if(!Command_queue.empty()) { 
    *val = Command_queue.front();
    Command_queue.pop();
    return true;
  }
  return false;
}

// true, if the queue is not empty
bool code_queue_data_available(){
  return !Command_queue.empty();
}

// clear the queue
void code_queue_clear(){
  while(!Command_queue.empty())
    Command_queue.pop();
}
