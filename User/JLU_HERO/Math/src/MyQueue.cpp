#include "MyQueue.hpp"

#include <cstdlib>
#include <cstring>

extern "C"
{

void Queue_Init(Queue *queue, const std::uint16_t maxSize)
{
    queue->data = nullptr;
    queue->maxSize = maxSize;
    queue->front = 0;
    queue->rear = 0;
    queue->buffer = nullptr;
    queue->bufElemSize = 0;

    if (maxSize == 0U)
    {
        return;
    }

    queue->data = static_cast<void **>(std::malloc(static_cast<std::size_t>(maxSize) * sizeof(void *)));

    if (queue->data != nullptr)
    {
        std::memset(queue->data, 0, static_cast<std::size_t>(maxSize) * sizeof(void *));
    }
}

void Queue_AttachBuffer(Queue *queue, void *buffer, const std::uint8_t elemSize)
{
    queue->buffer = buffer;
    queue->bufElemSize = elemSize;
}

void Queue_Destroy(Queue *queue)
{
    std::free(queue->data);
    queue->data = nullptr;
    queue->maxSize = 0U;
    queue->front = 0U;
    queue->rear = 0U;
    queue->buffer = nullptr;
    queue->bufElemSize = 0U;
}

std::uint16_t Queue_Size(Queue *queue)
{
    return static_cast<std::uint16_t>((queue->rear + queue->maxSize - queue->front) % queue->maxSize);
}

std::uint8_t Queue_IsFull(Queue *queue)
{
    return queue->front == ((queue->rear + 1U) % queue->maxSize) ? 1U : 0U;
}

std::uint8_t Queue_IsEmpty(Queue *queue)
{
    return queue->front == queue->rear ? 1U : 0U;
}

void *Queue_GetElement(Queue *queue, const std::uint16_t position)
{
    const std::uint16_t index = static_cast<std::uint16_t>((queue->front + position) % queue->maxSize);
    return queue->data[index];
}

void *Queue_Top(Queue *queue)
{
    return queue->data[queue->front];
}

void *Queue_Dequeue(Queue *queue)
{
    void *result = queue->data[queue->front];
    queue->front = static_cast<std::uint16_t>((queue->front + 1) % queue->maxSize);
    return result;
}

void Queue_Enqueue(Queue *queue, void *data)
{
    if (queue->buffer == nullptr)
    {
        // 只保存原始指针
        queue->data[queue->rear] = data;
    }
    else
    {
        // 将数据复制到绑定的缓存区
        auto *destination = static_cast<std::uint8_t *>(queue->buffer) + static_cast<std::size_t>(queue->rear) * queue->bufElemSize;

        std::memcpy(destination, data, queue->bufElemSize);

        queue->data[queue->rear] = destination;
    }

    queue->rear = static_cast<std::uint16_t>((queue->rear + 1U) % queue->maxSize);
}
}
