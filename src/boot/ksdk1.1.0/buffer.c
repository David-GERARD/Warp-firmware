#include <stdio.h>
#include <stdint.h> // For uint8_t, uint16_t
#include "buffer.h"




void CircularBuffer_Init(CircularBuffer *cb) {
    cb->head = 0;
}



uint16_t bufferGet(CircularBuffer *cb, int index) {
    if (index < 0) {
        // Simple error handling: return 0 if index is out of bounds
        return 0;
    }
    // Adjust index to account for head position and buffer wrapping
    int adjustedIndex = (cb->head - 1 - index) % BUFFER_SIZE;
    if (adjustedIndex < 0) {
        adjustedIndex += BUFFER_SIZE; // Correct negative index, if any
    }
    return cb->data[adjustedIndex];
}

void bufferAdd(CircularBuffer *cb, uint16_t value) {

    // Buffer is not full, simply add the value and increase the count
    cb->data[cb->head] = value;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
    return ;

}
