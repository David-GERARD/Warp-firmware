#define BUFFER_SIZE 75 // Example size, make sure to have enough space for LTA



typedef struct {
    int head; // position of last value added
    uint16_t data[BUFFER_SIZE];
} CircularBuffer;

void CircularBuffer_Init(CircularBuffer *cb);
uint16_t bufferGet(CircularBuffer *cb, int index);
void bufferAdd(CircularBuffer *cb, uint16_t value);


