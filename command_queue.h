// command_queue.h
// a simple <int> queue to store the commands for the RoverC

// add an int value to the end of the queue
void code_queue_add(int val);

// get the first value out of the queue
// return false if there is no element in the queue anymore
bool code_queue_get(int *val);

// true, if the queue is not empty
bool code_queue_data_available();

// clear the queue
void code_queue_clear();