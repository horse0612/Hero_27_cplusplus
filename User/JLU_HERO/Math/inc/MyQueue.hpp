#pragma once

#include <cstdint>

#define EMPTY_QUEUE {nullptr, 0U, 0U, 0U, nullptr, 0U}

struct Queue
{
    // 队列元素指针数组
    void **data;

    // 队列容量
    std::uint16_t maxSize;

    // 队头和队尾
    std::uint16_t front;
    std::uint16_t rear;

    // 可选的数据缓存区
    void *buffer;
    std::uint8_t bufElemSize;
};

extern "C"
{

void Queue_Init(Queue *queue, std::uint16_t maxSize);

void Queue_AttachBuffer(Queue *queue, void *buffer, std::uint8_t elemSize);

void Queue_Destroy(Queue *queue);

std::uint16_t Queue_Size(Queue *queue);

std::uint8_t Queue_IsFull(Queue *queue);

std::uint8_t Queue_IsEmpty(Queue *queue);

void *Queue_GetElement(Queue *queue, std::uint16_t position);

void *Queue_Top(Queue *queue);

void *Queue_Dequeue(Queue *queue);

void Queue_Enqueue(Queue *queue, void *data);
}